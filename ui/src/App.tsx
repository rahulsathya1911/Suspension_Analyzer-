import Hero from '@/components/ui/tailwind-css-background-snippet'

/**
 * App.tsx — ZOPHERIDAE dashboard shell
 *
 * Architecture:
 * - Hero wraps the entire viewport and provides the static radial gradient.
 *   It replaces the old WebGL DarkVeil canvas.
 * - The shell uses h-screen / overflow-hidden so panels scroll independently.
 * - Sidebar and card surfaces use `bg-card-glass` (rgba(17,17,17,0.55) +
 *   backdrop-blur-[14px]) so the gradient bleeds through.
 *
 * Migration path: paste screen content from Index.html into <main> below,
 * converting HTML → JSX incrementally.
 */
export default function App() {
  return (
    /* Hero takes h-full because the shell's body already fills the viewport */
    <Hero className="h-screen">
      <div className="flex h-full overflow-hidden">

        {/* ── Left Sidebar ── */}
        <aside className="flex w-[80px] shrink-0 flex-col border-r border-white/5 bg-card-glass shadow-2xl transition-all lg:w-[220px]">
          {/* Wordmark */}
          <div className="flex h-16 items-center justify-center border-b border-white/5 lg:justify-start lg:px-6">
            <span className="font-mono text-xl font-bold uppercase tracking-wider text-primary text-glow">
              ZOPHERIDAE
            </span>
          </div>

          {/* Nav items — placeholder until full migration */}
          <nav className="flex flex-1 flex-col gap-2 px-3 py-6 lg:px-4">
            {[
              { id: 'telemetry', label: 'Telemetry', active: true },
              { id: 'vehicle',   label: 'Vehicle Config', active: false },
              { id: 'diag',      label: 'Diagnostics', active: false },
            ].map(({ id, label, active }) => (
              <a
                key={id}
                href={`#${id}`}
                className={[
                  'flex items-center gap-3 rounded-xl px-3 py-3 font-mono text-xs uppercase tracking-wider transition-colors',
                  active
                    ? 'border border-primary/20 bg-primary/10 text-primary'
                    : 'text-muted-foreground hover:bg-white/5 hover:text-white',
                ].join(' ')}
              >
                <span className="hidden lg:block">{label}</span>
              </a>
            ))}
          </nav>
        </aside>

        {/* ── Main content area ── */}
        <main className="flex flex-1 flex-col overflow-hidden">
          {/* Top header bar */}
          <header className="flex h-16 shrink-0 items-center justify-between border-b border-white/5 bg-card-glass px-6">
            <h1 className="font-mono text-sm font-bold uppercase tracking-widest text-muted-foreground">
              Telemetry Dashboard
            </h1>
            <div className="flex items-center gap-2">
              <div className="h-2 w-2 animate-pulse rounded-full bg-success" />
              <span className="font-mono text-xs text-muted-foreground">LIVE</span>
            </div>
          </header>

          {/* Screen content — migrate Index.html tabs here */}
          <div className="flex flex-1 flex-col gap-4 overflow-auto p-6">
            {/* Placeholder card */}
            <div className="flex flex-1 items-center justify-center">
              <div className="bg-card-glass rounded-2xl border border-border/40 p-10 text-center">
                <p className="font-mono text-sm text-muted-foreground">
                  ✓ Hero gradient active — WebGL veil removed
                </p>
                <p className="mt-2 font-mono text-xs text-muted-foreground/60">
                  Migrate dashboard panels from Index.html into this shell
                </p>
                <div className="mt-4 inline-block rounded-lg bg-primary/10 px-4 py-2 font-mono text-sm text-primary">
                  gradient: #0a0a0a → #3a1214
                </div>
              </div>
            </div>
          </div>
        </main>

      </div>
    </Hero>
  )
}
