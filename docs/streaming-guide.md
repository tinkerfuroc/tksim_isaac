# Arena streaming and remote viewing

Reference for streaming the Tinker robot in the RoboCup Arena over WebRTC,
including the SSH tunnel path for client machines that cannot reach the
server directly over UDP, and client-side gotchas. See the README's ROS 2
Humble boundary section for the basic `launch-streaming` wrapper.

## Arena streaming

To stream the Tinker robot inside the committed RoboCup Arena 3 map instead of
opening an empty full Isaac UI, run:

```bash
./scripts/launch-arena-streaming
```

This standalone development viewer loads the current content-addressed
`robot.usd` and its colocated `map.yaml`, renders the map as visible collidable
walls, selects a deterministic arena overview camera, and listens on TCP 49100
and UDP 47998 for NVIDIA's WebRTC Streaming Client. The primary stream starts
at 1280x720 and uses Isaac Sim's supported dynamic-resize path to follow the
client window; spectator streams are not enabled. The launcher pumps Kit at a
bounded 10 Hz so WebRTC mouse, keyboard, and video are handled without coupling
the 120 Hz CPU-PhysX clock to ray-traced render latency; the guarded update
cannot step PhysX a second time. The occupancy-map cuboids are deliberately
kinematic static arena geometry, not loose props, so moving one in the stage
does not make it fall under gravity. It
also augments the robot's existing 20 kg low-mounted chassis ballast with 10 kg
(30 kg total, with proportionally scaled inertia). Base wheel velocity targets
are applied directly; navigation or another upstream controller is responsible
for acceleration and deceleration limits. It preserves Isaac's normal
single-session lifecycle: disconnecting the client
terminates the simulator and releases its ports. It
deliberately starts no external Humble/ROS processes; use the navigation
two-process workflow when ROS control is required. Optional launch arguments
such as `--duration 30` are forwarded to `launch-isaac`.

## Tunneling over SSH

When the client network cannot return UDP packets directly to tkserver, carry
both WebRTC transports over the existing SSH connection. Keep the arena server
running in its tkserver shell, then run this on the GUI/client machine:

```bash
./scripts/connect-arena-streaming tinker@tkserver.example.net
```

The SSH destination is required and can be a hostname, SSH configuration alias,
or `user@host`; there is no client-machine-specific `tkserver` default. The
launcher needs only Bash, Python 3, and OpenSSH on Linux or macOS. It fetches the
matching relay helper from the authenticated server into a private temporary
directory, runs it locally, and removes it on exit, so the client does not need
a Tinker Sim checkout. To install the single launcher on another machine:

```bash
scp tinker@tkserver.example.net:/home/tinker/tinker-sim/6.0.1/scripts/connect-arena-streaming .
chmod +x connect-arena-streaming
./connect-arena-streaming tinker@tkserver.example.net
```

Use `--ssh-port` and `--identity-file` for connections not fully described by
the local SSH configuration. `--remote-root` changes the server checkout path;
it defaults to `/home/tinker/tinker-sim/6.0.1`. Key- or agent-based SSH access
is required because the connector deliberately uses non-interactive
`BatchMode=yes`.

The connector forwards TCP signaling and preserves UDP datagram boundaries
while framing media packets over the SSH byte stream. It waits up to 180
seconds for a process- and port-validated readiness marker written only after
the arena, robot, viewport, and Kit input loop have initialized; override this
with `--ready-timeout`. Do not open the NVIDIA client until the connector prints
`SSH WebRTC tunnel ready`.

Keep the connector running and configure NVIDIA's native client with Server
`2130706433`, Signal `49100`, and Stream `47998`. `2130706433` is the IPv4
numeric form of `127.0.0.1`: it never leaves the client machine, needs no
hosts-file entry, stays IPv4-only, and avoids a client 2.0 bug where literal
`localhost` or dotted `127.x` makes the client omit the explicit media endpoint
and bypass the UDP tunnel. The server also advertises `127.0.0.1` as its fixed
WebRTC media address, ensuring ICE uses the SSH relay instead of tkserver's
physical interface. Streaming ports and loopback values can be overridden with
`--client-host`, `--local-bind`, `--signal-port`, and `--media-port`. Close the
client first, then press Ctrl-C in the tunnel shell. Because UDP is encapsulated
in TCP, packet loss can produce head-of-line delay; this path favors reliable
access over minimum streaming latency.

After connecting with NVIDIA's native client, move the pointer completely
outside the streamed video once and then move it back in before clicking. The
2.0 client enables mouse/keyboard forwarding on the video's pointer-enter
event; if the pointer remains over the Connect button while that form is
replaced by the video, the first clicks can remain local to the client instead
of being sent to Isaac Sim.
