
import { NextResponse } from 'next/server';

// IMPORTANT: Replace this with the actual IP address of Computer B.
const PYTHON_SERVER_IP = 'http://192.168.1.100'; // Example: 'http://192.168.1.123'

export async function POST(
  request: Request,
  { params }: { params: { direction: string } }
) {
  const direction = params.direction;

  if (!direction) {
    return NextResponse.json({ error: 'Direction not provided' }, { status: 400 });
  }

  try {
    const response = await fetch(`${PYTHON_SERVER_IP}:5000/move/${direction}`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    const data = await response.json();
    return NextResponse.json(data);

  } catch (error) {
    console.error('Error forwarding request to Python server:', error);
    // Make sure to check the console logs on your Next.js server for these errors.
    return NextResponse.json(
      { error: 'Failed to connect to the chair server.' },
      { status: 500 }
    );
  }
}
