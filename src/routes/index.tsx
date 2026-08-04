import { createFileRoute } from '@tanstack/react-router';
import { LauncherView } from '@/views/LauncherView';
import { RunSidebar } from '@/components/RunSidebar';

export const Route = createFileRoute('/')({
  component: Home,
});

function Home() {
  return (
    <div className="flex h-screen w-screen">
      <RunSidebar />
      <div className="min-w-0 flex-1">
        <LauncherView />
      </div>
    </div>
  );
}
