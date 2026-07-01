import { useState } from 'react';
import { Link, useNavigate } from '@tanstack/react-router';
import { Menu, Plus, LogOut, Crown, Settings, LayoutGrid } from 'lucide-react';
import { Button } from '@/components/ui/button';
import {
  Tooltip,
  TooltipContent,
  TooltipTrigger,
} from '@/components/ui/tooltip';
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuGroup,
  DropdownMenuItem,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from '@/components/ui/dropdown-menu';
import { useAuth } from '@/contexts/AuthContext';
import { supabase } from '@/lib/supabase';
import {
  Sheet,
  SheetContent,
  SheetDescription,
  SheetHeader,
  SheetTitle,
  SheetTrigger,
} from './ui/sheet';
import { useIsMobile } from '@/hooks/useIsMobile';
import { useQuery } from '@tanstack/react-query';
import { ConditionalWrapper } from './ConditionalWrapper';
import { cn } from '@/lib/utils';
import { Conversation, ConversationSettings } from '@shared/types';
import { UserAvatar } from '@/components/chat/UserAvatar';
import { useProfile } from '@/services/profileService';
import { apiJson } from '@/services/api';

interface SidebarProps {
  isSidebarOpen: boolean;
  setIsSidebarOpen: (open: boolean) => void;
}

// One maker2 conversation from /api/list-maker2-runs (a thread, or a legacy run).
type Maker2Run = {
  threadId: string;    // '' for legacy runs that predate threads
  run_dir: string;
  title: string;
  prompt: string;
  model: string;
  maxIters: number;
  created_at: string;
  ok: boolean;
  turns: number;
  judgePassed: boolean | null;
};

// A unified sidebar row: either a Supabase chat or a disk-backed maker2 run.
type SidebarItem =
  | { kind: 'chat'; id: string; title: string; when: string }
  | { kind: 'maker2'; id: string; title: string; when: string; run: Maker2Run };

type SidebarPath = '/' | '/history' | '/subscription';

function DesktopSidebar({ isSidebarOpen, setIsSidebarOpen }: SidebarProps) {
  const navigate = useNavigate();
  const { user, signOut } = useAuth();
  const isMobile = useIsMobile();
  const { data: profile } = useProfile();

  // Get 10 most recent conversations
  const { data: recentConversations } = useQuery<Conversation[]>({
    queryKey: ['conversations', 'recent'],
    initialData: [],
    queryFn: async () => {
      const { data, error } = await supabase
        .from('conversations')
        .select('*')
        .order('updated_at', { ascending: false })
        .eq('user_id', user?.id ?? '')
        .limit(10)
        .overrideTypes<Array<{ settings: ConversationSettings }>>();

      if (error) throw error;

      return data;
    },
  });

  // maker2 (articulated) runs live on disk, not in Supabase. Pull them and show
  // them in the SAME list as normal conversations; clicking one reopens the saved
  // run read-only. See /api/list-maker2-runs.
  const { data: maker2Runs } = useQuery<Maker2Run[]>({
    queryKey: ['maker2-runs', 'recent'],
    initialData: [],
    queryFn: async () => {
      const res = (await apiJson('list-maker2-runs')) as { runs?: Maker2Run[] };
      return res.runs ?? [];
    },
  });

  // Merge conversations + maker2 runs into one time-sorted list of sidebar items.
  const sidebarItems: SidebarItem[] = [
    ...(recentConversations ?? []).map((c): SidebarItem => ({
      kind: 'chat',
      id: c.id,
      title: c.title,
      when: c.updated_at ?? c.created_at ?? '',
    })),
    ...(maker2Runs ?? []).map((r): SidebarItem => ({
      kind: 'maker2',
      id: r.run_dir,
      title: r.title || 'Articulated run',
      when: r.created_at,
      run: r,
    })),
  ]
    .sort((a, b) => (b.when || '').localeCompare(a.when || ''))
    .slice(0, 12);

  const handleSignOut = async () => {
    try {
      await signOut();
      navigate({ to: '/signin' });
    } catch (error) {
      console.error('Error signing out:', error);
    }
  };

  const sidebarNavigate = (path: SidebarPath) => {
    if (isMobile) {
      setIsSidebarOpen(false); // setIsSidebarOpen is actually setOpen from Sheet component
    }
    navigate({ to: path });
  };

  const renderUserSectionTrigger = () => {
    if (isSidebarOpen) {
      return (
        <div className="flex cursor-pointer items-center space-x-3 rounded-md px-2 py-1.5 transition-colors hover:bg-accent-foreground">
          <UserAvatar />
          <div className="flex flex-col">
            <span className="text-sm font-medium text-adam-text-primary">
              {profile?.full_name || user?.email?.split('@')[0] || 'User'}
            </span>
            <span className="text-xs text-adam-text-tertiary dark:text-gray-400">
              {user?.email}
            </span>
          </div>
        </div>
      );
    }

    return (
      <Button
        variant="adam_dark_collapsed_avatar"
        className="group ml-[1px] h-[46px] w-[46px] px-0 py-6"
      >
        <UserAvatar className="h-[30px] w-[30px] transition-all duration-200 ease-in-out group-hover:h-[26px] group-hover:w-[26px] group-hover:ring-2 group-hover:ring-adam-neutral-500" />
      </Button>
    );
  };

  return (
    <div
      className={`${isSidebarOpen ? 'w-64' : 'w-16'} flex h-full flex-shrink-0 flex-col bg-adam-bg-dark pb-2 transition-all duration-300 ease-in-out dark:bg-gray-950`}
    >
      <div className="p-4 dark:border-gray-800">
        <ConditionalWrapper
          condition={!isSidebarOpen}
          wrapper={(children) => (
            <Tooltip>
              <TooltipTrigger asChild>{children}</TooltipTrigger>
              <TooltipContent side="right" className="flex flex-col">
                <span className="font-semibold">Home</span>
                <span className="text-xs text-muted-foreground">Home Page</span>
              </TooltipContent>
            </Tooltip>
          )}
        >
          <button
            type="button"
            className="flex w-full cursor-pointer items-center space-x-2"
            onClick={() => sidebarNavigate('/')}
          >
            {isSidebarOpen ? (
              <div className="flex w-full">
                <img
                  className="mx-auto h-8 w-full object-contain"
                  src={`${import.meta.env.BASE_URL}/automech-logo.png`}
                  alt="Logo"
                />
              </div>
            ) : (
              <img
                src={`${import.meta.env.BASE_URL}/automech-icon.png`}
                alt="Logo"
                className="h-8 w-8 min-w-8 object-contain"
              />
            )}
          </button>
        </ConditionalWrapper>
      </div>

      <div className="flex min-h-0 flex-1 flex-col">
        <div
          className={`${isSidebarOpen ? 'px-4' : 'px-2'} flex-1 py-2 transition-all duration-300 ease-in-out`}
        >
          <ConditionalWrapper
            condition={!isSidebarOpen}
            wrapper={(children) => (
              <Tooltip>
                <TooltipTrigger asChild>{children}</TooltipTrigger>
                <TooltipContent side="right" className="flex flex-col">
                  <span className="font-semibold">New Creation</span>
                  <span className="text-xs text-muted-foreground">
                    Start a new conversation
                  </span>
                </TooltipContent>
              </Tooltip>
            )}
          >
            <div className="ml-[9px]">
              <Button
                variant="secondary"
                className={` ${
                  isSidebarOpen
                    ? 'flex w-[216px] items-center justify-start gap-2 rounded-[100px] border border-adam-blue bg-adam-background-1 px-4 py-3 text-adam-text-primary hover:bg-adam-blue/40 hover:text-adam-text-primary'
                    : 'flex h-[30px] w-[30px] items-center justify-center rounded-[8px] border-2 border-adam-blue bg-white p-[2px] text-adam-text-primary shadow-[0px_4px_10px_0px_rgba(0,120,212,0.24)] hover:bg-adam-blue/40 hover:text-adam-text-primary'
                } mb-4`}
                onClick={() => sidebarNavigate('/')}
              >
                <Plus
                  className={`h-5 w-5 ${!isSidebarOpen ? 'text-adam-neutral-300 hover:text-adam-text-primary' : ''}`}
                />
                {isSidebarOpen && (
                  <div className="text-sm font-semibold leading-[14px] tracking-[-0.14px] text-adam-neutral-200">
                    New Creation
                  </div>
                )}
              </Button>
            </div>
          </ConditionalWrapper>
          <nav className="space-y-1">
            {[
              {
                icon: LayoutGrid,
                label: 'Creations',
                href: '/history' as const,
                description: 'View past creations',
                submenu: sidebarItems,
              },
            ].map(({ icon: Icon, label, href, description, submenu }) => (
              <div key={label} className="space-y-1">
                <ConditionalWrapper
                  condition={!isSidebarOpen}
                  wrapper={(children) => (
                    <Tooltip>
                      <TooltipTrigger asChild>{children}</TooltipTrigger>
                      <TooltipContent side="right" className="flex flex-col">
                        <span className="font-semibold">{label}</span>
                        <span className="text-xs text-muted-foreground">
                          {description}
                        </span>
                      </TooltipContent>
                    </Tooltip>
                  )}
                >
                  <Button
                    variant={
                      isSidebarOpen ? 'adam_dark' : 'adam_dark_collapsed'
                    }
                    onClick={() => sidebarNavigate(href)}
                    className={`${isSidebarOpen ? 'w-full justify-start' : 'ml-[1px] h-[46px] w-[46px] p-0'}`}
                  >
                    <Icon
                      className={`${isSidebarOpen ? 'mr-2' : ''} h-[22px] w-[22px] min-w-[22px]`}
                    />
                    {isSidebarOpen && label}
                  </Button>
                </ConditionalWrapper>
                {isSidebarOpen && submenu && (
                  <ul className="ml-7 flex list-none flex-col gap-1 border-l border-adam-neutral-500 px-2">
                    {submenu.map((item: SidebarItem) => {
                      const closeOnMobile = () => {
                        if (isMobile) setIsSidebarOpen(false);
                      };
                      const titleSpan = (
                        <li>
                          <span className="line-clamp-1 text-ellipsis text-nowrap rounded-md p-1 text-xs font-medium text-adam-neutral-400 transition-colors duration-200 ease-in-out [@media(hover:hover)]:hover:bg-adam-neutral-950 [@media(hover:hover)]:hover:text-adam-neutral-10">
                            {item.title}
                          </span>
                        </li>
                      );
                      // maker2 run -> reopen the saved run read-only (pass its dir).
                      if (item.kind === 'maker2') {
                        return (
                          <Link
                            key={item.id}
                            to="/maker2/$runId"
                            params={{
                              runId: encodeURIComponent(
                                item.run.threadId || item.run.run_dir,
                              ),
                            }}
                            search={
                              item.run.threadId
                                ? {
                                    prompt: item.run.prompt,
                                    model: item.run.model,
                                    iters: item.run.maxIters,
                                    thread: item.run.threadId,
                                  }
                                : {
                                    prompt: item.run.prompt,
                                    model: item.run.model,
                                    iters: item.run.maxIters,
                                    dir: item.run.run_dir,
                                  }
                            }
                            onClick={closeOnMobile}
                          >
                            {titleSpan}
                          </Link>
                        );
                      }
                      return (
                        <Link
                          key={item.id}
                          to="/editor/$id"
                          params={{ id: item.id }}
                          onClick={closeOnMobile}
                        >
                          {titleSpan}
                        </Link>
                      );
                    })}
                  </ul>
                )}
              </div>
            ))}
          </nav>
        </div>

        <div
          className={`${isSidebarOpen ? 'px-4' : 'px-2'} py-4 transition-all duration-300 ease-in-out dark:border-gray-800`}
        >
          <div className={cn('flex flex-col gap-2', isSidebarOpen && 'gap-3')}>
            <DropdownMenu>
              <DropdownMenuTrigger asChild>
                {renderUserSectionTrigger()}
              </DropdownMenuTrigger>
              <DropdownMenuContent
                className="w-56"
                align="end"
                side={isMobile ? 'top' : 'right'}
              >
                <div className="flex items-center space-x-2 p-2">
                  <div className="flex flex-col space-y-1">
                    <p className="text-sm font-medium text-adam-text-primary">
                      {profile?.full_name ||
                        user?.email?.split('@')[0] ||
                        'User'}
                    </p>
                    <p className="text-xs text-muted-foreground">
                      {user?.email}
                    </p>
                  </div>
                </div>
                <DropdownMenuSeparator />
                <DropdownMenuGroup className="text-adam-text-primary">
                  <DropdownMenuItem asChild>
                    <Link to="/settings" className="flex items-center">
                      <Settings className="mr-2 h-4 w-4" />
                      <span>Settings</span>
                    </Link>
                  </DropdownMenuItem>
                  <DropdownMenuItem
                    onClick={() => sidebarNavigate('/subscription')}
                  >
                    <Crown className="mr-2 h-4 w-4" />
                    <span>Subscriptions</span>
                  </DropdownMenuItem>
                </DropdownMenuGroup>
                <DropdownMenuSeparator />
                <DropdownMenuItem onClick={handleSignOut}>
                  <LogOut className="mr-2 h-4 w-4 text-adam-text-primary" />
                  <span className="text-adam-text-primary">Sign out</span>
                </DropdownMenuItem>
              </DropdownMenuContent>
            </DropdownMenu>
          </div>
        </div>
      </div>
    </div>
  );
}

function MobileSidebar({
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  isSidebarOpen,
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  setIsSidebarOpen,
}: SidebarProps) {
  const [open, setOpen] = useState(false);

  return (
    <Sheet open={open} onOpenChange={setOpen}>
      <SheetTrigger asChild>
        <Button
          variant="ghost"
          size="icon"
          className="fixed left-2 top-2.5 z-50 hover:bg-adam-neutral-700 md:hidden"
        >
          <Menu className="h-5 w-5 text-adam-text-primary" />
        </Button>
      </SheetTrigger>
      <SheetContent
        side="left"
        className="bg-adam-bg-dark p-0 [&>button]:text-adam-text-primary"
      >
        {/* For aria stuff */}
        <SheetHeader className="hidden">
          <SheetTitle className="text-adam-text-primary">AutoMech</SheetTitle>
          <SheetDescription>
            AI-powered CAD software for everyone
          </SheetDescription>
        </SheetHeader>
        <DesktopSidebar isSidebarOpen={true} setIsSidebarOpen={setOpen} />
      </SheetContent>
    </Sheet>
  );
}

export function Sidebar({ isSidebarOpen, setIsSidebarOpen }: SidebarProps) {
  const isMobile = useIsMobile();
  const { user } = useAuth();

  // Don't display the sidebar if the user isn't logged in
  if (user == null) {
    return <></>;
  }

  return isMobile ? (
    <MobileSidebar
      isSidebarOpen={isSidebarOpen}
      setIsSidebarOpen={setIsSidebarOpen}
    />
  ) : (
    <DesktopSidebar
      isSidebarOpen={isSidebarOpen}
      setIsSidebarOpen={setIsSidebarOpen}
    />
  );
}
