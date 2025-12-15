import React from 'react';
import { useAuth } from './AuthProvider';

interface ProtectedRouteProps {
  children: React.ReactNode;
  fallback?: React.ReactNode;
}

const ProtectedRoute: React.FC<ProtectedRouteProps> = ({ 
  children, 
  fallback = <div>Please log in to access this content.</div> 
}) => {
  const { state } = useAuth();

  if (state.loading) {
    return <div>Loading...</div>;
  }

  return state.isLoggedIn ? <>{children}</> : <>{fallback}</>;
};

export default ProtectedRoute;