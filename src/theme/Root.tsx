import React from 'react';
import { AuthProvider } from '../components/auth/AuthProvider';

// This component wraps the entire application with the AuthProvider
export default function Root({ children }: { children: React.ReactNode }) {
  return <AuthProvider>{children}</AuthProvider>;
}