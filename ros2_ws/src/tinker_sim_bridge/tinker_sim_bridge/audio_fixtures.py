from __future__ import annotations

import json
import threading
from collections import defaultdict, deque
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from tinker_audio_msgs.action import Doorbell, Listen
from tinker_audio_msgs.srv import (
    GetConfirmation,
    QuestionAnswer,
    TextToSpeech,
    WaitForStart,
)


class AudioFixtures(Node):
    """Deterministic non-acoustic servers driven by scenario dialogue."""

    def __init__(self) -> None:
        super().__init__("tinker_sim_audio_fixtures")
        self.declare_parameter("scenario_file", "")
        self._lock = threading.Lock()
        self._dialogue: dict[str, deque[str]] = defaultdict(deque)
        path = Path(str(self.get_parameter("scenario_file").value))
        if path.is_file():
            raw = json.loads(path.read_text(encoding="utf-8"))
            for entry in raw.get("dialogue", []):
                self._dialogue[str(entry["endpoint"])].append(str(entry["outcome"]))
        self._events = self.create_publisher(String, "/sim/fixtures/audio/events", 20)
        callbacks = ReentrantCallbackGroup()
        self._listen = ActionServer(
            self,
            Listen,
            "listen_action",
            execute_callback=self._listen_execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            callback_group=callbacks,
        )
        self._doorbell = ActionServer(
            self,
            Doorbell,
            "doorbell_action",
            execute_callback=self._doorbell_execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            callback_group=callbacks,
        )
        self.create_service(TextToSpeech, "announce", self._announce)
        self.create_service(WaitForStart, "wait_for_start", self._wait_for_start)
        self.create_service(
            QuestionAnswer, "question_answer_service", self._question_answer
        )
        self.create_service(
            GetConfirmation, "get_confirmation_service", self._confirmation
        )

    def _next(self, endpoint: str, default: str) -> str:
        with self._lock:
            queue = self._dialogue[endpoint]
            return queue.popleft() if queue else default

    def _event(self, endpoint: str, value: str) -> None:
        message = String()
        message.data = json.dumps(
            {"endpoint": endpoint, "value": value}, sort_keys=True
        )
        self._events.publish(message)

    def _listen_execute(self, goal_handle):
        value = self._next("listen_action", "")
        feedback = Listen.Feedback()
        feedback.progress = 1.0
        feedback.status_message = "deterministic scenario fixture"
        feedback.partial_transcription = value
        goal_handle.publish_feedback(feedback)
        result = Listen.Result()
        result.status = 0 if value else 1
        result.error_message = "" if value else "scenario dialogue exhausted"
        result.message = value
        if value:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self._event("listen_action", value)
        return result

    def _doorbell_execute(self, goal_handle):
        value = self._next("doorbell_action", "doorbell")
        result = Doorbell.Result()
        result.status = 0
        result.error_message = ""
        result.transcription = value
        goal_handle.succeed()
        self._event("doorbell_action", value)
        return result

    def _announce(self, request, response):
        self._event("announce", request.text)
        response.status = 0
        return response

    def _wait_for_start(self, _request, response):
        response.status = 0
        response.error_message = ""
        response.message = self._next("wait_for_start", "start")
        self._event("wait_for_start", response.message)
        return response

    def _question_answer(self, _request, response):
        response.answer = self._next("question_answer_service", "")
        response.status = 0 if response.answer else 1
        response.error_message = (
            "" if response.answer else "scenario dialogue exhausted"
        )
        self._event("question_answer_service", response.answer)
        return response

    def _confirmation(self, _request, response):
        value = self._next("get_confirmation_service", "yes").strip().lower()
        response.confirmed = value in {"yes", "true", "confirmed", "1"}
        response.status = 0
        response.error_message = ""
        self._event("get_confirmation_service", value)
        return response


def main() -> None:
    rclpy.init()
    node = AudioFixtures()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
