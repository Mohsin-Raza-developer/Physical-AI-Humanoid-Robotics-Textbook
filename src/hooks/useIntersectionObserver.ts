import { useEffect, useRef, useState } from 'react';

interface UseIntersectionObserverOptions {
  threshold?: number | number[];
  root?: Element | null;
  rootMargin?: string;
  triggerOnce?: boolean;
}

/**
 * Custom hook for scroll-triggered animations using IntersectionObserver API
 *
 * @param options - IntersectionObserver configuration options
 * @returns [ref, isIntersecting] - Ref to attach to element and intersection state
 *
 * @example
 * ```tsx
 * const [ref, isVisible] = useIntersectionObserver({ threshold: 0.1, triggerOnce: true });
 * return <div ref={ref} className={isVisible ? 'animate-fade-in' : 'opacity-0'}>Content</div>;
 * ```
 */
export function useIntersectionObserver<T extends Element = HTMLElement>({
  threshold = 0.1,
  root = null,
  rootMargin = '0px',
  triggerOnce = true,
}: UseIntersectionObserverOptions = {}): [React.RefObject<T>, boolean] {
  const ref = useRef<T>(null);
  const [isIntersecting, setIsIntersecting] = useState(false);

  useEffect(() => {
    const element = ref.current;
    if (!element) return;

    // Check if IntersectionObserver is supported
    if (!('IntersectionObserver' in window)) {
      // Fallback: always show content if IntersectionObserver is not supported
      setIsIntersecting(true);
      return;
    }

    const observer = new IntersectionObserver(
      ([entry]) => {
        const isElementIntersecting = entry.isIntersecting;

        if (isElementIntersecting) {
          setIsIntersecting(true);

          // If triggerOnce is true, disconnect observer after first intersection
          if (triggerOnce && observer) {
            observer.disconnect();
          }
        } else if (!triggerOnce) {
          // If triggerOnce is false, update state on exit as well
          setIsIntersecting(false);
        }
      },
      {
        threshold,
        root,
        rootMargin,
      }
    );

    observer.observe(element);

    return () => {
      if (observer) {
        observer.disconnect();
      }
    };
  }, [threshold, root, rootMargin, triggerOnce]);

  return [ref, isIntersecting];
}

export default useIntersectionObserver;
