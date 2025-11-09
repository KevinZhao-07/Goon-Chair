"use client";

import { useState, useCallback, useRef } from "react";
import { CustomCursor } from "@/components/CustomCursor";
import { ParticleEffect } from "@/components/ParticleEffect";
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
  const [findPersonActive, setFindPersonActive] = useState(false);
  const particleIdRef = useRef(0);



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

  const handleFindPerson = useCallback((e: React.MouseEvent<HTMLButtonElement>) => {
    handleButtonClick(e);
    setFindPersonActive((prev) => !prev);
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
      
      {/* YouTube Background */}
      <div className="absolute top-0 left-0 w-full h-full z-0 overflow-hidden">
        <iframe
          className="absolute top-1/2 left-1/2 w-full h-full -translate-x-1/2 -translate-y-1/2"
          src="https://www.youtube.com/embed/5vfaDsMhCF4?autoplay=1&mute=1&loop=1&playlist=5vfaDsMhCF4&controls=0&showinfo=0&autohide=1&modestbranding=1&iv_load_policy=3"
          frameBorder="0"
          allow="autoplay; encrypted-media"
          allowFullScreen
          style={{
            minWidth: '100%',
            minHeight: '100%',
            width: 'auto',
            height: 'auto',
            aspectRatio: '16 / 9',
          }}
        ></iframe>
        <div className="absolute inset-0 bg-black/10" />
      </div>

      {/* Title */}
      <div className="absolute top-8 left-8 z-20">
        <h1 className="text-6xl font-extrabold text-white" style={{ textShadow: '0 0 10px rgba(255,255,255,0.7), 0 0 20px rgba(255,255,255,0.5), 0 0 30px rgba(255,255,255,0.3)' }}>
            The Goon Chair
        </h1>
      </div>

      {/* Content - All at bottom */}
      <div className="relative z-10 min-h-screen flex flex-col justify-end">
        {/* All Controls at Bottom */}
        <div className="pb-8 px-4 md:px-6 space-y-4">
          {/* Control Buttons */}
          <div className="flex gap-4 justify-center">
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
              label="Find Person"
              onClick={handleFindPerson}
              active={findPersonActive}
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

