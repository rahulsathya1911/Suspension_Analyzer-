import type { ReactNode } from 'react'
import { cn } from '@/lib/utils'

interface HeroProps {
  /** Additional Tailwind classes — use `h-full` when nested inside a parent flex container */
  className?: string
  /** Content rendered above the gradient in a `relative z-10` wrapper */
  children?: ReactNode
}

/**
 * ZOPHERIDAE Hero Background
 *
 * Static radial gradient: dark #0a0a0a core → softened red edge (#3a1214).
 * Replaces the WebGL DarkVeil canvas.
 *
 * Design decisions:
 * - End stop uses #3a1214 (≈ #E84045 at ~25% saturation) so the red doesn't
 *   burn behind the translucent dashboard cards. Swap to #E84045 for full intensity.
 * - `-z-10` keeps the gradient strictly behind all UI siblings.
 * - `inset-0 absolute` — works correctly when a `relative` ancestor constrains it.
 * - `h-screen` default; pass `h-full` via className when used as an app wrapper
 *   inside an overflow-hidden shell.
 */
export function Hero({ className, children }: HeroProps) {
  return (
    <div className={cn('relative', className ?? 'h-screen')}>
      {/* Gradient layer — sits behind all children */}
      <div
        aria-hidden="true"
        className={cn(
          'absolute inset-0 -z-10',
          'bg-[radial-gradient(ellipse_at_top_right,_#3a1214_0%,_#0a0a0a_40%,_#0a0a0a_100%)]',
        )}
      />

      {/* Content slot — always above the gradient */}
      {children && (
        <div className="relative z-10 h-full">
          {children}
        </div>
      )}
    </div>
  )
}

export default Hero
