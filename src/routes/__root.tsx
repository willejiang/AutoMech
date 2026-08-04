import {
  createRootRoute,
  HeadContent,
  Outlet,
  Scripts,
} from '@tanstack/react-router';
import appCss from '@/index.css?url';

export const Route = createRootRoute({
  head: () => ({
    meta: [
      { charSet: 'utf-8' },
      { name: 'viewport', content: 'width=device-width, initial-scale=1' },
      { title: 'AutoMech' },
    ],
    links: [
      { rel: 'stylesheet', href: appCss },
      { rel: 'icon', type: 'image/svg+xml', href: '/automech-icon.svg' },
      { rel: 'icon', type: 'image/x-icon', href: '/automech-icon.ico' },
    ],
  }),
  component: RootComponent,
  errorComponent: ({ error }) => (
    <RootDocument>
      <div className="p-8 font-mono text-sm text-red-400">
        <div className="mb-2 font-semibold">Something broke</div>
        <pre className="whitespace-pre-wrap">{String(error)}</pre>
      </div>
    </RootDocument>
  ),
});

function RootComponent() {
  return (
    <RootDocument>
      <Outlet />
    </RootDocument>
  );
}

function RootDocument({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en">
      <head>
        <HeadContent />
      </head>
      <body>
        {children}
        <Scripts />
      </body>
    </html>
  );
}
