// Logging and metrics utility for the authentication system

export interface LogEntry {
  timestamp: Date;
  level: 'info' | 'warn' | 'error' | 'debug';
  module: string;
  operation: string;
  userId?: string;
  ip?: string;
  userAgent?: string;
  details?: any;
}

class Logger {
  private static instance: Logger;
  private logLevel: 'info' | 'warn' | 'error' | 'debug' = 'info';

  private constructor() {}

  public static getInstance(): Logger {
    if (!Logger.instance) {
      Logger.instance = new Logger();
    }
    return Logger.instance;
  }

  public setLogLevel(level: 'info' | 'warn' | 'error' | 'debug'): void {
    this.logLevel = level;
  }

  private shouldLog(level: 'info' | 'warn' | 'error' | 'debug'): boolean {
    const levels = { debug: 0, info: 1, warn: 2, error: 3 };
    const currentLevel = levels[this.logLevel];
    const messageLevel = levels[level];
    return messageLevel >= currentLevel;
  }

  public log(entry: LogEntry): void {
    if (!this.shouldLog(entry.level)) {
      return;
    }

    // Format log message
    const logMessage = {
      timestamp: entry.timestamp.toISOString(),
      level: entry.level.toUpperCase(),
      module: entry.module,
      operation: entry.operation,
      userId: entry.userId,
      ip: entry.ip,
      userAgent: entry.userAgent,
      details: entry.details
    };

    // Output to console (in production, this could be sent to a logging service)
    console.log(JSON.stringify(logMessage));
  }

  public info(module: string, operation: string, details?: any, userId?: string, ip?: string, userAgent?: string): void {
    this.log({
      timestamp: new Date(),
      level: 'info',
      module,
      operation,
      details,
      userId,
      ip,
      userAgent
    });
  }

  public warn(module: string, operation: string, details?: any, userId?: string, ip?: string, userAgent?: string): void {
    this.log({
      timestamp: new Date(),
      level: 'warn',
      module,
      operation,
      details,
      userId,
      ip,
      userAgent
    });
  }

  public error(module: string, operation: string, details?: any, userId?: string, ip?: string, userAgent?: string): void {
    this.log({
      timestamp: new Date(),
      level: 'error',
      module,
      operation,
      details,
      userId,
      ip,
      userAgent
    });
  }

  public debug(module: string, operation: string, details?: any, userId?: string, ip?: string, userAgent?: string): void {
    if (this.logLevel === 'debug') {
      this.log({
        timestamp: new Date(),
        level: 'debug',
        module,
        operation,
        details,
        userId,
        ip,
        userAgent
      });
    }
  }
}

// Create metrics tracking
export class MetricsTracker {
  private static instance: MetricsTracker;
  private metrics: Map<string, number> = new Map();

  private constructor() {}

  public static getInstance(): MetricsTracker {
    if (!MetricsTracker.instance) {
      MetricsTracker.instance = new MetricsTracker();
    }
    return MetricsTracker.instance;
  }

  public increment(key: string): void {
    const current = this.metrics.get(key) || 0;
    this.metrics.set(key, current + 1);
  }

  public decrement(key: string): void {
    const current = this.metrics.get(key) || 0;
    this.metrics.set(key, Math.max(0, current - 1));
  }

  public set(key: string, value: number): void {
    this.metrics.set(key, value);
  }

  public get(key: string): number | undefined {
    return this.metrics.get(key);
  }

  public getAll(): Map<string, number> {
    return new Map(this.metrics);
  }

  public getMetricsForAPIEndpoint(): Record<string, number> {
    const result: Record<string, number> = {};
    for (const [key, value] of this.metrics.entries()) {
      result[key] = value;
    }
    return result;
  }
}

// Predefined loggers for different modules
export const authLogger = {
  register: (userId?: string, ip?: string, userAgent?: string, details?: any) => 
    Logger.getInstance().info('auth', 'register', details, userId, ip, userAgent),
  
  login: (userId?: string, ip?: string, userAgent?: string, details?: any) => 
    Logger.getInstance().info('auth', 'login', details, userId, ip, userAgent),
  
  logout: (userId?: string, ip?: string, userAgent?: string, details?: any) => 
    Logger.getInstance().info('auth', 'logout', details, userId, ip, userAgent),
  
  profileAccess: (userId?: string, ip?: string, userAgent?: string, details?: any) => 
    Logger.getInstance().info('auth', 'profile', details, userId, ip, userAgent),
  
  passwordResetRequest: (userId?: string, ip?: string, userAgent?: string, details?: any) => 
    Logger.getInstance().info('auth', 'password-reset-request', details, userId, ip, userAgent),
  
  passwordResetComplete: (userId?: string, ip?: string, userAgent?: string, details?: any) => 
    Logger.getInstance().info('auth', 'password-reset-complete', details, userId, ip, userAgent),
  
  accountDeletion: (userId?: string, ip?: string, userAgent?: string, details?: any) => 
    Logger.getInstance().info('auth', 'account-deletion', details, userId, ip, userAgent),
  
  error: (operation: string, userId?: string, ip?: string, userAgent?: string, details?: any) => 
    Logger.getInstance().error('auth', operation, details, userId, ip, userAgent),
};

// Predefined metrics for authentication endpoints
export const authMetrics = {
  registerAttempt: () => MetricsTracker.getInstance().increment('auth.register.attempts'),
  registerSuccess: () => MetricsTracker.getInstance().increment('auth.register.success'),
  registerFailure: () => MetricsTracker.getInstance().increment('auth.register.failure'),
  
  loginAttempt: () => MetricsTracker.getInstance().increment('auth.login.attempts'),
  loginSuccess: () => MetricsTracker.getInstance().increment('auth.login.success'),
  loginFailure: () => MetricsTracker.getInstance().increment('auth.login.failure'),
  
  logoutAttempt: () => MetricsTracker.getInstance().increment('auth.logout.attempts'),
  
  profileAccess: () => MetricsTracker.getInstance().increment('auth.profile.access'),
  
  passwordResetRequestAttempt: () => MetricsTracker.getInstance().increment('auth.password-reset.request.attempts'),
  passwordResetRequestSuccess: () => MetricsTracker.getInstance().increment('auth.password-reset.request.success'),
  passwordResetCompleteAttempt: () => MetricsTracker.getInstance().increment('auth.password-reset.complete.attempts'),
  passwordResetCompleteSuccess: () => MetricsTracker.getInstance().increment('auth.password-reset.complete.success'),
  
  accountDeletionAttempt: () => MetricsTracker.getInstance().increment('auth.account-deletion.attempts'),
  accountDeletionSuccess: () => MetricsTracker.getInstance().increment('auth.account-deletion.success'),
  
  rateLimitHit: (endpoint: string) => MetricsTracker.getInstance().increment(`auth.rate-limit.${endpoint}`),
};

