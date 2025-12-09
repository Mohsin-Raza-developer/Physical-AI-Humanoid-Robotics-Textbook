import React from 'react';
import useIntersectionObserver from '../../hooks/useIntersectionObserver';

export type AnimationType = 'fade-in' | 'slide-up' | 'scale-in';

interface AnimatedSectionProps {
  children: React.ReactNode;
  animation?: AnimationType;
  delay?: number;
  className?: string;
  threshold?: number;
  triggerOnce?: boolean;
}

const animationClasses: Record<AnimationType, string> = {
  'fade-in': 'animate-fade-in',
  'slide-up': 'animate-slide-up',
  'scale-in': 'animate-scale-in',
};

/**
 * Wrapper component for scroll-triggered animations
 *
 * @param children - React elements to animate
 * @param animation - Type of animation ('fade-in', 'slide-up', 'scale-in')
 * @param delay - Animation delay in milliseconds (0-1000)
 * @param className - Additional CSS classes
 * @param threshold - Intersection threshold (0-1)
 * @param triggerOnce - Whether animation should only trigger once
 *
 * @example
 * ```tsx
 * <AnimatedSection animation="fade-in" delay={200}>
 *   <FeatureCard />
 * </AnimatedSection>
 * ```
 */
export default function AnimatedSection({
  children,
  animation = 'fade-in',
  delay = 0,
  className = '',
  threshold = 0.1,
  triggerOnce = true,
}: AnimatedSectionProps): JSX.Element {
  const [ref, isVisible] = useIntersectionObserver({
    threshold,
    triggerOnce,
    rootMargin: '0px 0px -50px 0px', // Trigger slightly before element enters viewport
  });

  const animationClass = animationClasses[animation];
  const delayStyle = delay > 0 ? { animationDelay: `${delay}ms` } : undefined;

  return (
    <div
      ref={ref}
      className={`
        ${isVisible ? animationClass : 'opacity-0'}
        ${className}
      `.trim()}
      style={delayStyle}
    >
      {children}
    </div>
  );
}
