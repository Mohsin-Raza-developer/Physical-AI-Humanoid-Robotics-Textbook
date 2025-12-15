import { Resend } from 'resend';
import { PasswordResetToken } from '../types/auth';

// Initialize Resend with the API key from environment
const resend = process.env.RESEND_API_KEY ? new Resend(process.env.RESEND_API_KEY) : null;

export interface PasswordResetEmailData {
  email: string;
  resetToken: string;
  userName?: string;
}

/**
 * Sends a password reset email to the user
 * @param data Contains email, resetToken, and optional userName
 */
export async function sendPasswordResetEmail(data: PasswordResetEmailData): Promise<{ success: boolean; message: string }> {
  if (!resend) {
    console.error('Resend not initialized: RESEND_API_KEY is not set in environment');
    return {
      success: false,
      message: 'Email service not configured. Please contact administrators.'
    };
  }

  try {
    // Construct the reset link using the base URL from environment
    // Construct the reset link using the site URL (Frontend)
    const siteUrl = process.env.NEXT_PUBLIC_SITE_URL || 'http://localhost:3000/Physical-AI-Humanoid-Robotics-Textbook';
    const resetUrl = `${siteUrl}/auth/reset-password?token=${encodeURIComponent(data.resetToken)}`;

    const { error } = await resend.emails.send({
      from: process.env.SENDER_EMAIL || 'noreply@yourdomain.com',
      to: data.email,
      subject: 'Password Reset Request',
      html: `
        <div style="font-family: Arial, sans-serif; max-width: 600px; margin: 0 auto;">
          <h2>Password Reset Request</h2>
          <p>Hello${data.userName ? ` ${data.userName}` : ''},</p>
          
          <p>You have requested to reset your password for the Physical AI & Humanoid Robotics Textbook portal.</p>
          
          <p>Please click the button below to reset your password:</p>
          
          <div style="text-align: center; margin: 30px 0;">
            <a href="${resetUrl}" 
               style="background-color: #4F46E5; color: white; padding: 12px 24px; 
                      text-decoration: none; border-radius: 6px; font-weight: bold; display: inline-block;">
              Reset Password
            </a>
          </div>
          
          <p>If the button doesn't work, copy and paste this link into your browser:</p>
          <p style="word-break: break-all;">${resetUrl}</p>
          
          <p><strong>Note:</strong> This link will expire in 1 hour for security reasons.</p>
          
          <p>If you didn't request a password reset, please ignore this email or contact our support team.</p>
          
          <br/>
          <p>Best regards,<br/>
          The Physical AI & Humanoid Robotics Team</p>
          
          <hr style="margin: 30px 0; border: 0; border-top: 1px solid #eaeaea;" />
          <small>
            This email was sent to ${data.email} as a registered user of the Physical AI & Humanoid Robotics Textbook portal.
            If you believe you received this email in error, please contact us.
          </small>
        </div>
      `,
      text: `
        Password Reset Request
        
        Hello${data.userName ? ` ${data.userName}` : ''},
        
        You have requested to reset your password for the Physical AI & Humanoid Robotics Textbook portal.
        
        Please click the link below to reset your password:
        ${resetUrl}
        
        Note: This link will expire in 1 hour for security reasons.
        
        If you didn't request a password reset, please ignore this email or contact our support team.
        
        Best regards,
        The Physical AI & Humanoid Robotics Team
        
        ---
        This email was sent to ${data.email} as a registered user of the Physical AI & Humanoid Robotics Textbook portal.
        If you believe you received this email in error, please contact us.
      `,
    });

    if (error) {
      console.error('Error sending password reset email:', error);
      return {
        success: false,
        message: error.message || 'Failed to send password reset email'
      };
    }

    console.log(`Password reset email sent successfully to ${data.email}`);
    return {
      success: true,
      message: 'Password reset email sent successfully'
    };
  } catch (error: any) {
    console.error('Unexpected error sending password reset email:', error);
    return {
      success: false,
      message: error?.message || 'An unexpected error occurred while sending the password reset email'
    };
  }
}

export interface WelcomeEmailData {
  email: string;
  userName?: string;
}

/**
 * Sends a welcome email to a newly registered user
 * @param data Contains email and optional userName
 */
export async function sendWelcomeEmail(data: WelcomeEmailData): Promise<{ success: boolean; message: string }> {
  if (!resend) {
    console.error('Resend not initialized: RESEND_API_KEY is not set in environment');
    return {
      success: false,
      message: 'Email service not configured. Please contact administrators.'
    };
  }

  try {
    const { error } = await resend.emails.send({
      from: process.env.SENDER_EMAIL || 'noreply@yourdomain.com',
      to: data.email,
      subject: 'Welcome to Physical AI & Humanoid Robotics Textbook!',
      html: `
        <div style="font-family: Arial, sans-serif; max-width: 600px; margin: 0 auto;">
          <h2>Welcome to Physical AI & Humanoid Robotics Textbook!</h2>
          <p>Hello${data.userName ? ` ${data.userName}` : ''},</p>
          
          <p>We're excited to have you join our community of learners exploring the intersection of AI and robotics.</p>
          
          <p>Your account has been successfully created. You can now access our interactive content and resources.</p>
          
          <p>If you have any questions or need assistance, feel free to reach out to our support team.</p>
          
          <div style="text-align: center; margin: 30px 0;">
            <a href="${process.env.NEXT_PUBLIC_SITE_URL || 'http://localhost:3000'}" 
               style="background-color: #4F46E5; color: white; padding: 12px 24px; 
                      text-decoration: none; border-radius: 6px; font-weight: bold; display: inline-block;">
              Visit Dashboard
            </a>
          </div>
          
          <br/>
          <p>Best regards,<br/>
          The Physical AI & Humanoid Robotics Team</p>
          
          <hr style="margin: 30px 0; border: 0; border-top: 1px solid #eaeaea;" />
          <small>
            This email was sent to ${data.email} as a registered user of the Physical AI & Humanoid Robotics Textbook portal.
            If you believe you received this email in error, please contact us.
          </small>
        </div>
      `,
      text: `
        Welcome to Physical AI & Humanoid Robotics Textbook!
        
        Hello${data.userName ? ` ${data.userName}` : ''},
        
        We're excited to have you join our community of learners exploring the intersection of AI and robotics.
        
        Your account has been successfully created. You can now access our interactive content and resources.
        
        If you have any questions or need assistance, feel free to reach out to our support team.
        
        Best regards,
        The Physical AI & Humanoid Robotics Team
        
        ---
        This email was sent to ${data.email} as a registered user of the Physical AI & Humanoid Robotics Textbook portal.
        If you believe you received this email in error, please contact us.
      `,
    });

    if (error) {
      console.error('Error sending welcome email:', error);
      return {
        success: false,
        message: error.message || 'Failed to send welcome email'
      };
    }

    console.log(`Welcome email sent successfully to ${data.email}`);
    return {
      success: true,
      message: 'Welcome email sent successfully'
    };
  } catch (error: any) {
    console.error('Unexpected error sending welcome email:', error);
    return {
      success: false,
      message: error?.message || 'An unexpected error occurred while sending the welcome email'
    };
  }
}

export interface AccountDeletedEmailData {
  email: string;
  userName?: string;
}

/**
 * Sends a confirmation email when account is deleted
 * @param data Contains email and optional userName
 */
export async function sendAccountDeletedEmail(data: AccountDeletedEmailData): Promise<{ success: boolean; message: string }> {
  if (!resend) {
    console.error('Resend not initialized: RESEND_API_KEY is not set in environment');
    return {
      success: false,
      message: 'Email service not configured. Please contact administrators.'
    };
  }

  try {
    const { error } = await resend.emails.send({
      from: process.env.SENDER_EMAIL || 'noreply@yourdomain.com',
      to: data.email,
      subject: 'Account Deletion Confirmation',
      html: `
        <div style="font-family: Arial, sans-serif; max-width: 600px; margin: 0 auto;">
          <h2>Account Deletion Confirmation</h2>
          <p>Hello${data.userName ? ` ${data.userName}` : ''},</p>
          
          <p>This email confirms that your account with the Physical AI & Humanoid Robotics Textbook portal has been successfully deleted.</p>
          
          <p>All your personal data has been permanently removed from our systems as per GDPR compliance requirements.</p>
          
          <p>If you have any questions or wish to rejoin our community in the future, please contact our support team.</p>
          
          <br/>
          <p>Best regards,<br/>
          The Physical AI & Humanoid Robotics Team</p>
          
          <hr style="margin: 30px 0; border: 0; border-top: 1px solid #eaeaea;" />
          <small>
            This email was sent to ${data.email} as a confirmation of account deletion.
            If you believe you received this email in error, please contact us.
          </small>
        </div>
      `,
      text: `
        Account Deletion Confirmation
        
        Hello${data.userName ? ` ${data.userName}` : ''},
        
        This email confirms that your account with the Physical AI & Humanoid Robotics Textbook portal has been successfully deleted.
        
        All your personal data has been permanently removed from our systems as per GDPR compliance requirements.
        
        If you have any questions or wish to rejoin our community in the future, please contact our support team.
        
        Best regards,
        The Physical AI & Humanoid Robotics Team
        
        ---
        This email was sent to ${data.email} as a confirmation of account deletion.
        If you believe you received this email in error, please contact us.
      `,
    });

    if (error) {
      console.error('Error sending account deletion email:', error);
      return {
        success: false,
        message: error.message || 'Failed to send account deletion confirmation email'
      };
    }

    console.log(`Account deletion confirmation email sent successfully to ${data.email}`);
    return {
      success: true,
      message: 'Account deletion confirmation email sent successfully'
    };
  } catch (error: any) {
    console.error('Unexpected error sending account deletion email:', error);
    return {
      success: false,
      message: error?.message || 'An unexpected error occurred while sending the account deletion confirmation email'
    };
  }
}

// Verify that the email service is properly configured
export function isEmailServiceConfigured(): boolean {
  return !!process.env.RESEND_API_KEY && !!process.env.SENDER_EMAIL;
}