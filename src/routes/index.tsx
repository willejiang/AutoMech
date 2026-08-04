import { createFileRoute } from '@tanstack/react-router';

export const Route = createFileRoute('/')({
  component: Landing,
});

function Landing() {
  return (
    <div className="flex min-h-screen items-center justify-center">
      <div className="text-center">
        <div className="text-2xl font-semibold tracking-tight">AutoMech</div>
        <div className="mt-2 text-sm text-muted-foreground">
          scaffold up — workbench lands next
        </div>
      </div>
    </div>
  );
}
