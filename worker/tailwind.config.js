/** @type {import('tailwindcss').Config} */
export default {
  darkMode: ['class'],
  content: [
    './pages/**/*.{ts,tsx}',
    './components/**/*.{ts,tsx}',
    './app/**/*.{ts,tsx}',
    './src/**/*.{ts,tsx}',
    './node_modules/streamdown/dist/**/*.{js,mjs}',
  ],
  safelist: ['w-6', 'w-7', 'w-8', 'w-9', 'w-10', 'w-11', 'w-12'],
  theme: {
    extend: {
      screens: {
        desktop: '936px',
      },
      borderRadius: {
        lg: 'var(--radius)',
        md: 'calc(var(--radius) - 2px)',
        sm: 'calc(var(--radius) - 4px)',
      },
      fontFamily: {
        sans: [
          '"Segoe UI"',
          '"Segoe UI Web (West European)"',
          'system-ui',
          'sans-serif',
        ],
        mono: ['"Cascadia Code"', '"Cascadia Mono"', 'monospace'],
        'dm-sans': ['"Segoe UI"', 'sans-serif'],
        'kumbh-sans': ['"Segoe UI"', 'sans-serif'],
      },
      colors: {
        'adam-bg-dark': '#F3F2F1',
        // ??? lol that's what its called in Figma!
        'adam-background-light': '#F1F1F1',
        'adam-bg-secondary-dark': '#FFFFFF',
        'adam-bg-light': '#E5E5E3',
        'adam-bg-secondary-light': '#ECECEB',
        'adam-blue': '#0078D4',
        'adam-blue-dark': '#106EBE',
        'adam-text-primary': '#201F1E',
        'adam-text-secondary': '#605E5C',
        'adam-text-tertiary': '#A19F9D',
        'secondary-tan': '#E5E5E3',
        'background-color': '#F3F2F1',
        'white-16%': 'rgba(0,0,0,0.06)',
        'white-700': '#323130',
        'white-500': '#605E5C',
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
        pink: '#2B88D8',
        'sidebar-color': '#FFFFFF',
        'bg-gray': 'rgba(243, 242, 241)',
        background: 'hsl(var(--background))',
        foreground: 'hsl(var(--foreground))',
        card: {
          DEFAULT: 'hsl(var(--card))',
          foreground: 'hsl(var(--card-foreground))',
        },
        popover: {
          DEFAULT: 'hsl(var(--popover))',
          foreground: 'hsl(var(--popover-foreground))',
        },
        primary: {
          DEFAULT: 'hsl(var(--primary))',
          foreground: 'hsl(var(--primary-foreground))',
        },
        secondary: {
          DEFAULT: 'hsl(var(--secondary))',
          foreground: 'hsl(var(--secondary-foreground))',
        },
        muted: {
          DEFAULT: 'hsl(var(--muted))',
          foreground: 'hsl(var(--muted-foreground))',
        },
        accent: {
          DEFAULT: 'hsl(var(--accent))',
          foreground: 'hsl(var(--accent-foreground))',
        },
        destructive: {
          DEFAULT: 'hsl(var(--destructive))',
          foreground: 'hsl(var(--destructive-foreground))',
        },
        border: 'hsl(var(--border))',
        input: 'hsl(var(--input))',
        ring: 'hsl(var(--ring))',
        chart: {
          1: 'hsl(var(--chart-1))',
          2: 'hsl(var(--chart-2))',
          3: 'hsl(var(--chart-3))',
          4: 'hsl(var(--chart-4))',
          5: 'hsl(var(--chart-5))',
        },
      },
      keyframes: {
        'accordion-down': {
          from: {
            height: '0',
          },
          to: {
            height: 'var(--radix-accordion-content-height)',
          },
        },
        'accordion-up': {
          from: {
            height: 'var(--radix-accordion-content-height)',
          },
          to: {
            height: '0',
          },
        },
        'dot-bounce-1': {
          '0%, 80%, 100%': { transform: 'translateY(0)' },
          '40%': { transform: 'translateY(-8px)' },
        },
        'dot-bounce-2': {
          '0%, 20%, 100%': { transform: 'translateY(0)' },
          '60%': { transform: 'translateY(-8px)' },
        },
        'dot-bounce-3': {
          '0%, 40%, 100%': { transform: 'translateY(0)' },
          '80%': { transform: 'translateY(-8px)' },
        },
      },
      animation: {
        'accordion-down': 'accordion-down 0.2s ease-out',
        'accordion-up': 'accordion-up 0.2s ease-out',
        'dot-bounce-1': 'dot-bounce-1 1.0s infinite ease-in-out',
        'dot-bounce-2': 'dot-bounce-2 1.0s infinite ease-in-out',
        'dot-bounce-3': 'dot-bounce-3 1.0s infinite ease-in-out',
      },
    },
  },
  plugins: [require('tailwindcss-animate')],
};
