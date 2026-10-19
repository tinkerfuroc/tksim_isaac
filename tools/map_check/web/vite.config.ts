import react from '@vitejs/plugin-react';
import { defineConfig } from 'vitest/config';

// `npm run dev` proxies the API and the rendered images to a running `map_check` server.
const target = 'http://127.0.0.1:9081';
export default defineConfig({
  plugins: [react()],
  server: { proxy: { '/api': { target }, '/renders': { target } } },
  build: { outDir: 'dist', emptyOutDir: true },
  test: { environment: 'node' },
});
