import { createFileRoute } from '@tanstack/react-router';
import { SettingsView } from '@/views/SettingsView';
import { RunSidebar } from '@/components/RunSidebar';

export const Route = createFileRoute('/settings')({
  component: SettingsPage,
});

function SettingsPage() {
  return (
    <div className="flex h-screen w-screen">
      <RunSidebar />
      <div className="min-w-0 flex-1 overflow-y-auto">
        <SettingsView />
      </div>
    </div>
  );
}
