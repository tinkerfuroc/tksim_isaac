import { afterEach, describe, expect, it, vi } from 'vitest';
import { ApiError, saveMap, startRender } from './api';

function respond(status: number, body: unknown, statusText = '') {
  const text = typeof body === 'string' ? body : JSON.stringify(body);
  vi.stubGlobal('fetch', vi.fn(async () => new Response(text, { status, statusText })));
}

afterEach(() => { vi.unstubAllGlobals(); });

describe('ApiError', () => {
  it('carries status, detail and busy_text from a 409 body', async () => {
    respond(409, { detail: 'GPU busy: isaac-sim on GPU 1', busy_text: 'isaac-sim is running on GPU 1' });
    const err = await startRender('m').catch((e: unknown) => e);
    expect(err).toBeInstanceOf(ApiError);
    expect(err).toMatchObject({
      status: 409, detail: 'GPU busy: isaac-sim on GPU 1', busy_text: 'isaac-sim is running on GPU 1',
    });
  });
  it('keeps a 409 detail verbatim and has no busy_text when the body has none', async () => {
    const detail = "a map named 'x' already exists in /maps";
    respond(409, { detail });
    const err = (await saveMap({ token: 't', name: 'x', wall_height: 0.6, shapes: {}, erased: [] })
      .catch((e: unknown) => e)) as ApiError;
    expect(err.status).toBe(409);
    expect(err.detail).toBe(detail);
    expect(err.message).toBe(detail);
    expect(err.busy_text).toBeUndefined();
  });
  it('falls back to the status line when the body is not JSON', async () => {
    respond(502, '<html>bad gateway</html>', 'Bad Gateway');
    const err = (await saveMap({ token: 't', name: 'x', wall_height: 0.6, shapes: {}, erased: [] })
      .catch((e: unknown) => e)) as ApiError;
    expect(err.status).toBe(502);
    expect(err.detail).toBe('502 Bad Gateway');
  });
});
