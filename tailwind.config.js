/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
    "./docs/**/*.{md,mdx}",
    "./blog/**/*.{md,mdx}",
    "./pages/**/*.{md,mdx}",
    "./docusaurus.config.{js,ts}",
  ],
  theme: {
    extend: {
      colors: {
        'physical-ai-blue': '#2563eb',
        'physical-ai-dark': '#1e293b',
        'physical-ai-accent': '#7c3aed',
      }
    },
  },
  plugins: [],
}