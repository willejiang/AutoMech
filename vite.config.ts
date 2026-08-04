import { tanstackStart } from '@tanstack/react-start/plugin/vite';
import { nitro } from 'nitro/vite';
import path from 'node:path';
import react from '@vitejs/plugin-react';
import { defineConfig } from 'vite';

// Served from the root, unlike the cadam app this was split out of (which lives under
// /cadam and therefore had every fetch prefixed). apiUrl() reads BASE_URL, so keeping
// the base at '/' is what makes '/api/run-maker2-stream' just work.
export default defineConfig({
  plugins: [
    tanstackStart({ spa: { enabled: true } }),
    // inlineDynamicImports is load-bearing: the SSE route spawns Python through
    // node:child_process, which Nitro will not bundle correctly when split.
    nitro({ inlineDynamicImports: true }),
    react(),
  ],
  resolve: {
    alias: { '@': path.resolve(import.meta.dirname, './src') },
  },
  build: {
    outDir: 'dist/client',
    emptyOutDir: true,
    chunkSizeWarningLimit: 1000,
  },
  environments: {
    server: { build: { outDir: 'dist/server' } },
  },
  server: { port: 3000, open: false },
  preview: { port: 4173, host: true },
  optimizeDeps: { exclude: ['three'] },
});
