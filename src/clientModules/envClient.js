// src/clientModules/envClient.js
// Client module to make environment variables available to the frontend

// This file is referenced in docusaurus.config.js under clientModules

// Add environment variables to the global window object for browser access
if (typeof window !== 'undefined') {
  window.env = {
    NEXT_PUBLIC_API_BASE_URL: process.env.NEXT_PUBLIC_API_BASE_URL || 'http://localhost:3002',
  };
}