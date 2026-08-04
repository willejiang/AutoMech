/** @type {import('tailwindcss').Config} */
export default {
  darkMode: ['class'],
  content: ['./src/**/*.{ts,tsx}'],
  theme: {
    extend: {
      colors: {
        // The 'adam' palette the ported workbench components reference by name
        // (146 usages across the views/panels). Without these the classes are
        // silently dropped by Tailwind and the UI renders unstyled.
        'adam-bg-dark': '#F3F2F1',
        'adam-background-light': '#F1F1F1',
        'adam-bg-secondary-dark': '#FFFFFF',
        'adam-bg-light': '#E5E5E3',
        'adam-bg-secondary-light': '#ECECEB',
        'adam-blue': '#0078D4',
        'adam-blue-dark': '#106EBE',
        'adam-text-primary': '#201F1E',
        'adam-text-secondary': '#605E5C',
        'adam-text-tertiary': '#A19F9D',
        'adam-background-1': '#FFFFFF',
        'adam-background-2': '#F3F2F1',
        'adam-neutral-950': '#FFFFFF',
        'adam-neutral-900': '#FAF9F8',
        'adam-neutral-800': '#F3F2F1',
        'adam-neutral-700': '#EDEBE9',
        'adam-neutral-500': '#C8C6C4',
        'adam-neutral-400': '#A19F9D',
        'adam-neutral-300': '#797775',
        'adam-neutral-200': '#605E5C',
        'adam-neutral-100': '#323130',
        'adam-neutral-50': '#252423',
        'adam-neutral-10': '#201F1E',
        'adam-neutral-0': '#1B1A19',
        border: 'hsl(var(--border))',
        input: 'hsl(var(--input))',
        ring: 'hsl(var(--ring))',
        background: 'hsl(var(--background))',
        foreground: 'hsl(var(--foreground))',
        primary: {
          DEFAULT: 'hsl(var(--primary))',
          foreground: 'hsl(var(--primary-foreground))',
        },
        secondary: {
          DEFAULT: 'hsl(var(--secondary))',
          foreground: 'hsl(var(--secondary-foreground))',
        },
        destructive: {
          DEFAULT: 'hsl(var(--destructive))',
          foreground: 'hsl(var(--destructive-foreground))',
        },
        muted: {
          DEFAULT: 'hsl(var(--muted))',
          foreground: 'hsl(var(--muted-foreground))',
        },
        accent: {
          DEFAULT: 'hsl(var(--accent))',
          foreground: 'hsl(var(--accent-foreground))',
        },
        popover: {
          DEFAULT: 'hsl(var(--popover))',
          foreground: 'hsl(var(--popover-foreground))',
        },
        card: {
          DEFAULT: 'hsl(var(--card))',
          foreground: 'hsl(var(--card-foreground))',
        },
        chart: {
          1: 'hsl(var(--chart-1))',
          2: 'hsl(var(--chart-2))',
          3: 'hsl(var(--chart-3))',
          4: 'hsl(var(--chart-4))',
          5: 'hsl(var(--chart-5))',
        },
      },
      borderRadius: {
        lg: 'var(--radius)',
        md: 'calc(var(--radius) - 2px)',
        sm: 'calc(var(--radius) - 4px)',
      },
    },
  },
  plugins: [import('tailwindcss-animate')],
};
