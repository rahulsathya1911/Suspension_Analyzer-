import { clsx, type ClassValue } from 'clsx'
import { twMerge } from 'tailwind-merge'

/**
 * Merges Tailwind class names safely, resolving conflicts with tailwind-merge.
 * Used by all shadcn/ui components and custom components.
 */
export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs))
}
