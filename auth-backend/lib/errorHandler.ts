import { NextApiHandler, NextApiRequest, NextApiResponse } from 'next';

// Type for standard error responses based on API contracts
interface ErrorResponse {
  success: boolean;
  message: string;
  errors?: Array<{
    field?: string;
    message: string;
  }>;
  timestamp?: string;
  path?: string;
}

/**
 * Error handling middleware function
 * This catches errors and formats them in a consistent way
 */
export const errorHandler = (
  fn: NextApiHandler
): NextApiHandler => {
  return async (req: NextApiRequest, res: NextApiResponse) => {
    try {
      await fn(req, res);
    } catch (error: any) {
      console.error(`Error in ${req.method} ${req.url}:`, error);

      // Determine the status code based on the type of error
      let statusCode = 500;
      let errorMessage = 'Internal server error';
      let errors: Array<{ field?: string; message: string }> = [];

      // Customize error responses based on error type
      if (error.type) {
        switch (error.type) {
          case 'VALIDATION_ERROR':
            statusCode = 400;
            errorMessage = error.message || 'Validation failed';
            if (error.details) {
              errors = error.details;
            }
            break;
          case 'AUTHENTICATION_ERROR':
            statusCode = 401;
            errorMessage = error.message || 'Authentication failed';
            break;
          case 'AUTHORIZATION_ERROR':
            statusCode = 403;
            errorMessage = error.message || 'Access denied';
            break;
          case 'NOT_FOUND':
            statusCode = 404;
            errorMessage = error.message || 'Resource not found';
            break;
          case 'CONFLICT':
            statusCode = 409;
            errorMessage = error.message || 'Conflict occurred';
            break;
          default:
            // Generic error
            break;
        }
      } else if (error.code) {
        // Database errors
        if (error.code === '23505') { // Unique violation in PostgreSQL
          statusCode = 409;
          errorMessage = 'A resource with this identifier already exists';
        } else if (error.code === '23503') { // Foreign key violation
          statusCode = 400;
          errorMessage = 'Invalid reference in request data';
        }
      }

      // Format the response based on API contract
      const response: ErrorResponse = {
        success: false,
        message: errorMessage,
        errors: errors.length > 0 ? errors : undefined,
        timestamp: new Date().toISOString(),
        path: req.url,
      };

      res.status(statusCode).json(response);
    }
  };
};

/**
 * Higher-order function that wraps API handlers with error handling
 */
export const withErrorHandling = (handler: NextApiHandler): NextApiHandler => {
  return errorHandler(handler);
};

/**
 * Standardized error response function for use within handlers
 */
export const sendErrorResponse = (
  res: NextApiResponse,
  statusCode: number,
  message: string,
  errors?: Array<{ field?: string; message: string }>
) => {
  const response: ErrorResponse = {
    success: false,
    message,
    errors,
    timestamp: new Date().toISOString(),
    path: res.getHeader('path') as string || 'unknown',
  };

  res.status(statusCode).json(response);
};

/**
 * Standardized success response function
 */
export const sendSuccessResponse = (
  res: NextApiResponse,
  data?: any,
  message?: string,
  statusCode: number = 200
) => {
  const response = {
    success: true,
    message: message || 'Success',
    data,
    timestamp: new Date().toISOString(),
  };

  res.status(statusCode).json(response);
};

/**
 * Middleware to handle specific error scenarios with consistent responses
 */
export const specificErrorHandler = (handler: NextApiHandler): NextApiHandler => {
  return async (req: NextApiRequest, res: NextApiResponse) => {
    try {
      await handler(req, res);
    } catch (error: any) {
      // Handle specific error types consistently
      if (error.name === 'ValidationError') {
        // Handle validation errors
        return sendErrorResponse(res, 400, 'Validation failed',
          error.errors?.map((err: any) => ({
            field: err.path,
            message: err.message
          })) || [{ message: error.message }]
        );
      } else if (error.name === 'UnauthorizedError') {
        // Handle unauthorized errors
        return sendErrorResponse(res, 401, 'Unauthorized');
      } else if (error.name === 'NotFoundError') {
        // Handle not found errors
        return sendErrorResponse(res, 404, 'Resource not found');
      } else if (error.code === 'ER_DUP_ENTRY' || error.code === '23505') {
        // Handle duplicate entry errors (MySQL and PostgreSQL)
        return sendErrorResponse(res, 409, 'Duplicate entry not allowed');
      } else {
        // For all other errors, use the general error handler
        console.error(`Unhandled error in ${req.method} ${req.url}:`, error);
        return sendErrorResponse(res, 500, 'Internal server error');
      }
    }
  };
};

/**
 * Global error logger middleware
 */
export const logErrors = (handler: NextApiHandler): NextApiHandler => {
  return async (req: NextApiRequest, res: NextApiResponse) => {
    try {
      return await handler(req, res);
    } catch (error: any) {
      // Log error details for debugging
      console.error({
        timestamp: new Date().toISOString(),
        method: req.method,
        url: req.url,
        headers: {
          'content-type': req.headers['content-type'],
          'user-agent': req.headers['user-agent'],
          'x-forwarded-for': req.headers['x-forwarded-for'],
        },
        body: req.body ? 'Present' : 'None', // Don't log the full body for privacy
        error: {
          message: error.message,
          stack: error.stack,
          type: error.constructor.name,
        }
      });

      throw error; // Re-throw to be handled by other middleware
    }
  };
};

/**
 * Combined error handling middleware chain
 */
export const baseErrorHandler = (handler: NextApiHandler): NextApiHandler => {
  return logErrors(specificErrorHandler(handler));
};