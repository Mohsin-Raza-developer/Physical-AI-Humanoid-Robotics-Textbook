import React, { createContext, useContext, useReducer, ReactNode, useEffect } from 'react';

interface AuthState {
  isLoggedIn: boolean;
  user: any | null;
  loading: boolean;
}

interface AuthAction {
  type: string;
  payload?: any;
}

interface AuthContextType {
  state: AuthState;
  login: (userData: any) => void;
  logout: () => void;
  updateUser: (userData: any) => void;
}

const initialState: AuthState = {
  isLoggedIn: typeof window !== 'undefined' ? !!localStorage.getItem('isLoggedIn') : false,
  user: typeof window !== 'undefined' ? JSON.parse(localStorage.getItem('user') || 'null') : null,
  loading: true,
};

const AuthContext = createContext<AuthContextType | undefined>(undefined);

const authReducer = (state: AuthState, action: AuthAction): AuthState => {
  switch (action.type) {
    case 'LOGIN_START':
      return { ...state, loading: true };
    case 'LOGIN_SUCCESS':
      return { ...state, isLoggedIn: true, user: action.payload, loading: false };
    case 'LOGOUT':
      return { ...state, isLoggedIn: false, user: null, loading: false };
    case 'UPDATE_USER':
      return { ...state, user: action.payload };
    case 'SET_LOADING':
      return { ...state, loading: action.payload };
    default:
      return state;
  }
};

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [state, dispatch] = useReducer(authReducer, initialState);

  useEffect(() => {
    // Check if user is logged in on initial load
    const checkLoginStatus = () => {
      const isLoggedIn = localStorage.getItem('isLoggedIn') === 'true';
      const userData = localStorage.getItem('user');

      if (isLoggedIn && userData) {
        try {
          const user = JSON.parse(userData);
          dispatch({
            type: 'LOGIN_SUCCESS',
            payload: user
          });
        } catch (e) {
          console.error('Failed to parse user data from localStorage:', e);
          dispatch({
            type: 'LOGOUT'
          });
        }
      } else {
        dispatch({
          type: 'SET_LOADING',
          payload: false
        });
      }
    };

    checkLoginStatus();
  }, []);

  const login = (userData: any) => {
    localStorage.setItem('isLoggedIn', 'true');
    localStorage.setItem('user', JSON.stringify(userData));
    if (userData.token) {
      localStorage.setItem('authToken', userData.token);
    }

    dispatch({
      type: 'LOGIN_SUCCESS',
      payload: userData
    });
  };

  const logout = () => {
    localStorage.removeItem('isLoggedIn');
    localStorage.removeItem('user');
    localStorage.removeItem('userId');
    localStorage.removeItem('email');
    localStorage.removeItem('authToken');

    dispatch({
      type: 'LOGOUT'
    });
  };

  const updateUser = (userData: any) => {
    localStorage.setItem('user', JSON.stringify(userData));

    dispatch({
      type: 'UPDATE_USER',
      payload: userData
    });
  };

  const value = {
    state,
    login,
    logout,
    updateUser
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};