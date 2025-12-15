import type { NextApiRequest, NextApiResponse } from 'next';
import { UserModel } from '../../../lib/user';
import { validateEmail, sanitizeInput } from '../../../lib/validation';
import { applySecurityHeaders } from '../../../lib/security';

export default async function handler(
    req: NextApiRequest,
    res: NextApiResponse
) {
    // Handle CORS
    if (req.method === 'OPTIONS') {
        const origin = req.headers.origin;
        const allowedOrigins = [
            process.env.CORS_ORIGIN || 'http://localhost:3000',
            'http://localhost:3000',
            'http://localhost:3001',
            'http://localhost:3002',
            'https://physical-ai-humanoid-robotics-textbook.github.io'
        ];

        if (origin && allowedOrigins.includes(origin)) {
            res.setHeader('Access-Control-Allow-Origin', origin);
        } else {
            res.setHeader('Access-Control-Allow-Origin', allowedOrigins[0]);
        }

        res.setHeader('Access-Control-Allow-Methods', 'POST, OPTIONS');
        res.setHeader('Access-Control-Allow-Headers', 'Content-Type');
        res.status(200).end();
        return;
    }

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
