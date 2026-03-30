/**
 * Basic web E2E tests for horus monitor.
 *
 * Verifies: page loads, correct title, no JS errors, key DOM elements exist,
 * API endpoints respond, status bar renders.
 */

const { MonitorTestHarness } = require('./harness');

let harness;

// Increase Jest/Node timeout for server startup
const TEST_TIMEOUT = 30000;

async function runTests() {
    const results = [];
    const failed = [];

    function test(name, fn) {
        results.push({ name, fn });
    }

    // ── Define tests ────────────────────────────────────────────────────

    test('page loads with correct title', async () => {
        const title = await harness.page.title();
        if (!title.includes('HORUS')) {
            throw new Error(`Expected title containing 'HORUS', got: '${title}'`);
        }
    });

    test('no console errors on page load', async () => {
        // Filter out expected warnings (like WebSocket close on test teardown)
        const realErrors = harness.consoleErrors.filter(e =>
            !e.includes('WebSocket') && !e.includes('favicon') &&
            !e.includes('Content Security Policy') && !e.includes('404') &&
            !e.includes('fonts.googleapis')
        );
        if (realErrors.length > 0) {
            throw new Error(`Console errors: ${realErrors.join(', ')}`);
        }
    });

    test('status bar is visible', async () => {
        // The status bar contains node count and topic count
        const body = await harness.page.evaluate(() => document.body.innerText);
        if (!body.includes('Nodes') && !body.includes('nodes')) {
            throw new Error('Status bar should mention Nodes');
        }
    });

    test('monitor tab exists', async () => {
        const body = await harness.page.evaluate(() => document.body.innerHTML);
        if (!body.includes('Monitor') && !body.includes('monitor')) {
            throw new Error('Should have Monitor tab or heading');
        }
    });

    test('API /api/status responds with JSON', async () => {
        const resp = await harness.page.evaluate(async () => {
            const r = await fetch('/api/status');
            return { status: r.status, body: await r.json() };
        });
        if (resp.status !== 200) {
            throw new Error(`/api/status returned ${resp.status}`);
        }
        if (typeof resp.body !== 'object') {
            throw new Error('/api/status should return JSON object');
        }
    });

    test('API /api/nodes responds with valid JSON', async () => {
        const resp = await harness.page.evaluate(async () => {
            const r = await fetch('/api/nodes');
            return { status: r.status, body: await r.json() };
        });
        if (resp.status !== 200) {
            throw new Error(`/api/nodes returned ${resp.status}`);
        }
        // May be array or object depending on discovery state
        if (typeof resp.body !== 'object') {
            throw new Error('/api/nodes should return JSON');
        }
    });

    test('API /api/topics responds with valid JSON', async () => {
        const resp = await harness.page.evaluate(async () => {
            const r = await fetch('/api/topics');
            return { status: r.status, body: await r.json() };
        });
        if (resp.status !== 200) {
            throw new Error(`/api/topics returned ${resp.status}`);
        }
        if (typeof resp.body !== 'object') {
            throw new Error('/api/topics should return JSON');
        }
    });

    test('API /api/graph responds with nodes and edges', async () => {
        const resp = await harness.page.evaluate(async () => {
            const r = await fetch('/api/graph');
            return { status: r.status, body: await r.json() };
        });
        if (resp.status !== 200) {
            throw new Error(`/api/graph returned ${resp.status}`);
        }
        if (!resp.body.nodes || !resp.body.edges) {
            throw new Error('/api/graph should have nodes and edges');
        }
    });

    test('JavaScript functions loaded', async () => {
        const fns = await harness.page.evaluate(() => {
            return {
                switchTab: typeof switchTab === 'function',
                updateStatus: typeof updateStatus === 'function',
                connectWebSocket: typeof connectWebSocket === 'function',
            };
        });
        const missing = Object.entries(fns).filter(([, v]) => !v).map(([k]) => k);
        if (missing.length > 0) {
            throw new Error(`Missing JS functions: ${missing.join(', ')}`);
        }
    });

    test('page renders within 5 seconds', async () => {
        const start = Date.now();
        await harness.page.reload({ waitUntil: 'domcontentloaded' });
        const elapsed = Date.now() - start;
        if (elapsed > 5000) {
            throw new Error(`Page took ${elapsed}ms to render (limit: 5000ms)`);
        }
        console.log(`  render time: ${elapsed}ms`);
    });

    // ── Run tests ───────────────────────────────────────────────────────

    console.log(`\nRunning ${results.length} web E2E tests...\n`);
    let passed = 0;

    for (const { name, fn } of results) {
        try {
            await fn();
            console.log(`  \u2713 ${name}`);
            passed++;
        } catch (err) {
            console.log(`  \u2717 ${name}: ${err.message}`);
            failed.push({ name, error: err.message });
        }
    }

    console.log(`\n${passed}/${results.length} passed, ${failed.length} failed\n`);
    return failed;
}

(async () => {
    harness = new MonitorTestHarness();
    let exitCode = 0;

    try {
        console.log('Starting monitor server + headless browser...');
        await harness.setup();
        console.log(`Server running on port ${harness.port}`);

        const failed = await runTests();
        if (failed.length > 0) {
            exitCode = 1;
        }
    } catch (err) {
        console.error('Setup failed:', err.message);
        if (harness.serverOutput) {
            console.error('Server output:', harness.serverOutput.slice(0, 500));
        }
        exitCode = 1;
    } finally {
        await harness.teardown();
    }

    process.exit(exitCode);
})();
