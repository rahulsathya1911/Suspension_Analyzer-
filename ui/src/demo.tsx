import Hero from '@/components/ui/tailwind-css-background-snippet'

/**
 * demo.tsx — visual smoke test for the Hero gradient background.
 * Import this instead of App.tsx temporarily to preview the gradient in isolation.
 *
 * Usage in main.tsx:
 *   import Demo from './demo'
 *   createRoot(...).render(<Demo />)
 */
export default function Demo() {
  return (
    <Hero className="h-screen">
      {/* ── Demo content ── */}
      <div className="flex h-full flex-col items-center justify-center gap-6 px-8 text-center">
        {/* Logo / wordmark */}
        <div className="font-mono text-4xl font-bold uppercase tracking-widest text-primary text-glow">
          ZOPHERIDAE
        </div>

        <p className="max-w-md font-mono text-sm text-muted-foreground">
          Hero gradient active — #0a0a0a core → #3a1214 edge.
          <br />
          WebGL DarkVeil removed. Static CSS gradient is the only background layer.
        </p>

        {/* Token swatch strip */}
        <div className="flex flex-wrap justify-center gap-3 text-xs font-mono">
          {[
            { label: '--background', hex: '#0a0a0a', tw: 'bg-background border border-border' },
            { label: '--card',       hex: '#121212', tw: 'bg-card border border-border' },
            { label: '--primary',    hex: '#E84045', tw: 'bg-primary' },
            { label: '--success',    hex: '#22c55e', tw: 'bg-success' },
            { label: '--warning',    hex: '#eab308', tw: 'bg-warning' },
          ].map(({ label, hex, tw }) => (
            <div key={label} className="flex items-center gap-2">
              <div className={`h-5 w-5 rounded-sm ${tw}`} />
              <span className="text-muted-foreground">
                {label} <span className="text-foreground">{hex}</span>
              </span>
            </div>
          ))}
        </div>

        {/* Glass card example */}
        <div className="bg-card-glass rounded-xl border border-border/40 p-6 text-left text-sm">
          <div className="mb-2 font-mono text-xs uppercase tracking-widest text-muted-foreground">
            Glass card (bg-card-glass)
          </div>
          <p className="text-foreground/80">
            Sidebar and dashboard cards should use{' '}
            <code className="rounded bg-primary/10 px-1 text-primary">bg-card-glass</code>{' '}
            so the gradient shows through via backdrop-blur.
          </p>
        </div>
      </div>
    </Hero>
  )
}
