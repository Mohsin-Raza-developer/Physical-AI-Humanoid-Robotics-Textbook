import React from 'react';
import { AppProps } from 'next/app'; // For Next.js apps
import { AuthProvider } from './components/auth/AuthProvider';

// For Docusaurus apps, we need to use a different approach
// This code will be used in the docusaurus.config.js as a theme component

export default function App({ Component, pageProps }: AppProps) {
  return (
    <AuthProvider>
      <Component {...pageProps} />
    </AuthProvider>
  );
}