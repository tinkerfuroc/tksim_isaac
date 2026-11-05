import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { subscribe } from './events';

/** Minimal EventSource stand-in: records every instance so a test can fail or open it. */
class FakeEventSource {
  static all: FakeEventSource[] = [];
  url: string;
  closed = false;
  private handlers: Record<string, ((e: unknown) => void)[]> = {};
  constructor(url: string) { this.url = url; FakeEventSource.all.push(this); }
  addEventListener(type: string, fn: (e: unknown) => void) { (this.handlers[type] ??= []).push(fn); }
  close() { this.closed = true; }
  emit(type: string, data?: string) { (this.handlers[type] ?? []).forEach((fn) => fn({ data })); }
}

beforeEach(() => {
  FakeEventSource.all = [];
  vi.useFakeTimers();
  vi.stubGlobal('EventSource', FakeEventSource);
});
afterEach(() => {
  vi.useRealTimers();
  vi.unstubAllGlobals();
});

const last = () => FakeEventSource.all[FakeEventSource.all.length - 1];

/** Fail the current connection and return how long until the next one opens (ms). */
function failAndMeasure(): number {
  const before = FakeEventSource.all.length;
  last().emit('error');
  let waited = 0;
  while (FakeEventSource.all.length === before) {
    vi.advanceTimersByTime(100);
    waited += 100;
    if (waited > 60_000) throw new Error('never reconnected');
  }
  return waited;
}

describe('subscribe', () => {
  it('reconnects with doubling backoff capped at 10 s, and resets after a good open', () => {
    const close = subscribe('m', () => {}, () => {});
    expect(last().url).toBe('/api/events?map=m');
    const waits = Array.from({ length: 7 }, failAndMeasure);
    expect(waits).toEqual([1000, 2000, 4000, 8000, 10000, 10000, 10000]);
    last().emit('open');
    expect(failAndMeasure()).toBe(1000);
    close();
  });
  it('stops reconnecting once closed', () => {
    const close = subscribe('m', () => {}, () => {});
    last().emit('error');
    close();
    vi.advanceTimersByTime(60_000);
    expect(FakeEventSource.all).toHaveLength(1);
  });
  it('passes bench versions and job frames through, ignoring bad frames', () => {
    const bench = vi.fn();
    const job = vi.fn();
    const close = subscribe('a b', bench, job);
    expect(last().url).toBe('/api/events?map=a%20b');
    last().emit('bench', '7');
    last().emit('bench', 'x');
    last().emit('job', '{"id":"j1"}');
    last().emit('job', '{not json');
    expect(bench.mock.calls).toEqual([[7]]);
    expect(job.mock.calls).toEqual([[{ id: 'j1' }]]);
    close();
    expect(last().closed).toBe(true);
  });
});
