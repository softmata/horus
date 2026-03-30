/**
 * Test harness for horus monitor web E2E tests.
 *
 * Starts the horus monitor server on a random port, launches headless
 * Chromium via Puppeteer, and provides helpers for page interaction.
 * Works in WSL2 without a display server.
 */

const puppeteer = require('puppeteer');
const { spawn } = require('child_process');
const net = require('net');
const path = require('path');

/** Find a free TCP port */
function getFreePort() {
    return new Promise((resolve, reject) => {
        const srv = net.createServer();
        srv.listen(0, '127.0.0.1', () => {
            const port = srv.address().port;
            srv.close(() => resolve(port));
        });
        srv.on('error', reject);
    });
}

/** Wait until a TCP port accepts connections */
async function waitForPort(port, timeoutMs = 15000) {
    const start = Date.now();
    while (Date.now() - start < timeoutMs) {
        try {
            await new Promise((resolve, reject) => {
                const sock = net.createConnection(port, '127.0.0.1');
                sock.on('connect', () => { sock.destroy(); resolve(); });
                sock.on('error', reject);
            });
            return;
        } catch {
            await new Promise(r => setTimeout(r, 200));
        }
    }
    throw new Error(`Port ${port} not ready after ${timeoutMs}ms`);
}

/** Find the horus binary */
function findHorusBin() {
    const candidates = [
        path.resolve(__dirname, '../../../target/debug/horus'),
        path.resolve(__dirname, '../../../target/release/horus'),
        path.resolve(__dirname, '../../target/debug/horus'),
        path.resolve(__dirname, '../../target/release/horus'),
    ];
    for (const p of candidates) {
        try {
            require('fs').accessSync(p, require('fs').constants.X_OK);
            return p;
        } catch {}
    }
    throw new Error('horus binary not found. Run cargo build first.');
}

class MonitorTestHarness {
    constructor() {
        this.server = null;
        this.browser = null;
        this.page = null;
        this.port = null;
        this.consoleErrors = [];
    }

    /** Start monitor server + headless browser */
    async setup() {
        this.port = await getFreePort();
        const horusBin = findHorusBin();

        // Start horus monitor with --no-auth on the free port (port is positional arg)
        this.server = spawn(horusBin, ['monitor', '--no-auth', String(this.port)], {
            env: { ...process.env, RUST_LOG: 'warn' },
            stdio: ['ignore', 'pipe', 'pipe'],
        });

        // Collect server output for debugging
        this.serverOutput = '';
        this.server.stdout.on('data', d => this.serverOutput += d.toString());
        this.server.stderr.on('data', d => this.serverOutput += d.toString());

        // Wait for server to be ready
        await waitForPort(this.port);

        // Launch headless browser
        this.browser = await puppeteer.launch({
            headless: true,
            args: [
                '--no-sandbox',
                '--disable-setuid-sandbox',
                '--disable-dev-shm-usage',
                '--disable-gpu',
            ],
        });

        this.page = await this.browser.newPage();

        // Collect console errors
        this.page.on('console', msg => {
            if (msg.type() === 'error') {
                this.consoleErrors.push(msg.text());
            }
        });

        // Navigate to monitor (use domcontentloaded — WebSocket keeps networkidle active)
        await this.page.goto(`http://127.0.0.1:${this.port}`, {
            waitUntil: 'domcontentloaded',
            timeout: 10000,
        });
        // Give JS a moment to initialize
        await new Promise(r => setTimeout(r, 1000));
    }

    /** Clean up server + browser */
    async teardown() {
        if (this.page) {
            try { await this.page.close(); } catch {}
        }
        if (this.browser) {
            try { await this.browser.close(); } catch {}
        }
        if (this.server) {
            this.server.kill('SIGTERM');
            // Give it a moment to die
            await new Promise(r => setTimeout(r, 500));
            if (!this.server.killed) {
                this.server.kill('SIGKILL');
            }
        }
    }

    get url() {
        return `http://127.0.0.1:${this.port}`;
    }
}

module.exports = { MonitorTestHarness, getFreePort, waitForPort };
