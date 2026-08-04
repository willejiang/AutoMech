import { createRouter as createTanStackRouter } from '@tanstack/react-router';

import { routeTree } from './routeTree.gen';

type AppRouter = ReturnType<typeof createAppRouter>;

let clientRouter: AppRouter | undefined;

// No basepath: this app is served from the root. (The cadam app it was split out of
// lived under /cadam, which is why every fetch there carried that prefix.)
function createAppRouter() {
  return createTanStackRouter({
    routeTree,
    defaultPreload: 'intent',
    scrollRestoration: true,
  });
}

export function getRouter() {
  if (typeof window !== 'undefined') {
    clientRouter ??= createAppRouter();
    return clientRouter;
  }

  return createAppRouter();
}

declare module '@tanstack/react-router' {
  interface Register {
    router: ReturnType<typeof getRouter>;
  }
}
