import type { NextApiRequest, NextApiResponse } from 'next';
import { UserModel } from '../../../lib/user';
import { validateEmail, sanitizeInput } from '../../../lib/validation';
import { applySecurityHeaders } from '../../../lib/security';
import { withCors } from '../../../middleware/cors';

async function handler(
    req: NextApiRequest,
    res: NextApiResponse
) {
    applySecurityHeaders(req, res);

    if (req.method !== 'POST') {
        return res.status(405).json({ message: 'Method not allowed' });
    }

    try {
        const { email } = sanitizeInput(req.body);

        if (!email) {
            return res.status(400).json({ message: 'Email is required' });
        }

        const validation = validateEmail(email);
        if (!validation.isValid) {
            return res.status(400).json({ message: 'Invalid email format' });
        }

        const existingUser = await UserModel.findByEmail(email);

        return res.status(200).json({
            exists: !!existingUser,
            message: existingUser ? 'Email already registered' : 'Email available'
        });

    } catch (error: any) {
        console.error('Check email error:', error);
        return res.status(500).json({ message: 'Internal server error' });
    }
}

export default withCors(handler);
