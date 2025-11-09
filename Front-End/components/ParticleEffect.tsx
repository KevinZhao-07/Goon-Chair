"use client";

import { useEffect, useRef, memo } from "react";

interface ParticleEffectProps {
  id: number;
  x: number;
  y: number;
  onComplete: () => void;
}

function ParticleEffectComponent({ id, x, y, onComplete }: ParticleEffectProps) {
  const particlesContainerRef = useRef<HTMLDivElement>(null); // Renamed for clarity
  const timeoutRef = useRef<NodeJS.Timeout | null>(null);
  const particleElementsRef = useRef<HTMLDivElement[]>([]); // Renamed for clarity
  const mountedRef = useRef(true);

  useEffect(() => {
    mountedRef.current = true;
    const container = particlesContainerRef.current; // Use the new ref
    if (!container) return;

    // No longer clearing children here, as each ParticleEffect instance is independent.
    particleElementsRef.current = [];

    const particleCount = 20 + Math.floor(Math.random() * 20); // Randomize particle count
    const newParticleElements: HTMLDivElement[] = []; // Renamed for clarity

    const cleanup = () => {
      if (!mountedRef.current) return;
      particleElementsRef.current.forEach(particle => {
        try {
          if (particle && particle.parentNode) {
            particle.parentNode.removeChild(particle);
          }
        } catch (error) {
          // Ignore cleanup errors
        }
      });
      particleElementsRef.current = [];
    };

    try {
      for (let i = 0; i < particleCount; i++) {
        const particle = document.createElement("div");
        particle.className = "particle";
        particle.setAttribute("data-particle-id", `${id}-${i}`);
        
        const angle = Math.random() * Math.PI * 2; // Random angle
        const velocity = 50 + Math.random() * 100; // Random velocity
        const tx = Math.cos(angle) * velocity;
        const ty = Math.sin(angle) * velocity; // Add vertical randomness
        const randomDelay = Math.random() * 0.2; // Shorter delay
        const duration = 0.8 + Math.random() * 0.8; // Random duration
        
        particle.style.position = "absolute"; // Changed to absolute relative to its container
        particle.style.left = `${x}px`;
        particle.style.top = `${y}px`;
        particle.style.setProperty("--tx", `${tx}px`);
        particle.style.setProperty("--ty", `${ty}px`);
        particle.style.animationDelay = `${randomDelay}s`;
        particle.style.animationDuration = `${duration}s`; // Set random duration
        const size = 3 + Math.random() * 9; // Random size
        particle.style.width = `${size}px`;
        particle.style.height = `${size}px`;
        particle.style.boxShadow = "0 0 10px rgba(255, 255, 255, 0.8)";
        particle.style.zIndex = "50";
        
        container.appendChild(particle);
        newParticleElements.push(particle);
      }

      particleElementsRef.current = newParticleElements;

      timeoutRef.current = setTimeout(() => {
        if (mountedRef.current) {
          cleanup();
          onComplete();
        }
      }, 1800); // Increased timeout to account for longer animation durations
    } catch (error) {
      console.error("Error creating particles:", error);
      cleanup();
      if (mountedRef.current) {
        onComplete();
      }
    }

    return () => {
      mountedRef.current = false;
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
        timeoutRef.current = null;
      }
      cleanup();
    };
  }, [x, y, onComplete, id]);

  return <div ref={particlesContainerRef} className="fixed inset-0 pointer-events-none z-50" data-particle-container={id} />;
}

export const ParticleEffect = memo(ParticleEffectComponent);

