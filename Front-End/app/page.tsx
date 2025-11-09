"use client";

import { useState, useCallback, useRef } from "react";
import { CustomCursor } from "@/components/CustomCursor";
import { ParticleEffect } from "@/components/ParticleEffect";
import { SoundButton } from "@/components/SoundButton";
import { ControlButton } from "@/components/ControlButton";

interface Particle {
  id: number;
  x: number;
  y: number;
}

export default function Home() {
  const [particles, setParticles] = useState<Particle[]>([]);
  const [trackPersonActive, setTrackPersonActive] = useState(false);
  const [gooningMachineActive, setGooningMachineActive] = useState(false);
  const particleIdRef = useRef(0);

  const soundLabels = [
    "Sound 1",
    "Sound 2",
    "Sound 3",
    "Sound 4",
    "Sound 5",
    "Sound 6",
    "Sound 7",
    "Sound 8",
    "Sound 9",
    "Sound 10",
  ];

  const handleButtonClick = useCallback((e: React.MouseEvent<HTMLElement>) => {
    try {
      // Capture coordinates immediately
      const rect = e.currentTarget.getBoundingClientRect();
      const x = rect.left + rect.width / 2;
      const y = rect.top + rect.height / 2;

      // Clear all existing particles first
      setParticles([]);
      
      // Use setTimeout to ensure cleanup completes before new animation starts
      setTimeout(() => {
        particleIdRef.current += 1;
        const id = particleIdRef.current;
        // Start new particle animation
        setParticles([{ id, x, y }]);
      }, 0);
    } catch (error) {
      console.error("Error handling button click:", error);
    }
  }, []);

  const handleParticleComplete = useCallback((id: number) => {
    setParticles((prev) => prev.filter((p) => p.id !== id));
  }, []);

  const handleTrackPerson = useCallback((e: React.MouseEvent<HTMLButtonElement>) => {
    handleButtonClick(e);
    setTrackPersonActive((prev) => !prev);
  }, [handleButtonClick]);

  const handleGooningMachine = useCallback((e: React.MouseEvent<HTMLButtonElement>) => {
    handleButtonClick(e);
    setGooningMachineActive((prev) => !prev);
  }, [handleButtonClick]);

  // New handler for sending movement commands
  const handleMove = useCallback(async (direction: string) => {
    console.log(`Sending command: ${direction}`);
    try {
      const response = await fetch(`/api/chair/${direction}`, {
        method: 'POST',
      });
      const result = await response.json();
      if (result.status === 'error') {
        console.error(`Error from chair server: ${result.message}`);
      } else {
        console.log(`Chair server response:`, result);
      }
    } catch (error) {
      console.error('Failed to send command to Next.js API route:', error);
    }
  }, []);


  return (
    <main className="custom-cursor min-h-screen relative overflow-hidden bg-black">
      <CustomCursor />
      
      {/* Livestream Background - Full Screen */}
      <div className="absolute inset-0 z-0">
        <div className="absolute inset-0 bg-black/10" />
      </div>

      {/* Content - All at bottom */}
      <div className="relative z-10 min-h-screen flex flex-col justify-end">
        {/* All Controls at Bottom */}
        <div className="pb-4 px-4 md:px-6 space-y-2.5">
          {/* Title - Minimal */}
          <div className="mb-3">
            <h1 className="text-xl md:text-2xl font-light text-white/80">
              The Goon Chair
            </h1>
          </div>

          {/* Sound Buttons - Minimal */}
          <div className="flex flex-wrap gap-1.5 justify-center">
            {soundLabels.map((label, index) => (
              <SoundButton
                key={index}
                label={label}
                onPlay={handleButtonClick}
              />
            ))}
          </div>

          {/* Control Buttons */}
          <div className="flex gap-2 justify-center">
            <ControlButton
              label="Stop"
              onClick={(e) => { handleButtonClick(e); handleMove('stop'); }}
            />
            <ControlButton
              label="Track Person"
              onClick={handleTrackPerson}
              active={trackPersonActive}
            />
            <ControlButton
              label="Gooning Machine"
              onClick={handleGooningMachine}
              active={gooningMachineActive}
            />
          </div>
        </div>
      </div>

      {/* Particle Effects */}
      {particles.map((particle) => (
        <ParticleEffect
          key={particle.id}
          id={particle.id}
          x={particle.x}
          y={particle.y}
          onComplete={() => handleParticleComplete(particle.id)}
        />
      ))}
    </main>
  );
}

