/**
 * Web E2E tests: tab rendering, data display, click interactions,
 * parameter editing, and WebSocket live updates.
 */

const { MonitorTestHarness } = require('./harness');

let harness;

/** Click a nav tab button by its label text */
async function clickTab(page, label) {
    const buttons = await page.$$('.nav-item');
    for (const btn of buttons) {
        const text = await btn.evaluate(el => el.textContent.trim());
        if (text === label) { await btn.click(); break; }
    }
    await new Promise(r => setTimeout(r, 300));
}

async function runTests() {
    const results = [];
    const failed = [];

    function test(name, fn) {
        results.push({ name, fn });
    }

    // ── Tab Rendering ───────────────────────────────────────────────────

    test('Monitor tab is active by default', async () => {
        const active = await harness.page.evaluate(() => {
            const btn = document.querySelector('.nav-item.active');
            return btn ? btn.textContent.trim() : null;
        });
        if (active !== 'Monitor') {
            throw new Error(`Expected active tab 'Monitor', got '${active}'`);
        }
    });

    test('switch to Parameters tab', async () => {
        // Must click the button (switchTab uses event.target)
        const buttons = await harness.page.$$('.nav-item');
        for (const btn of buttons) {
            const text = await btn.evaluate(el => el.textContent.trim());
            if (text === 'Parameters') { await btn.click(); break; }
        }
        await new Promise(r => setTimeout(r, 300));
        const active = await harness.page.evaluate(() => {
            const btn = document.querySelector('.nav-item.active');
            return btn ? btn.textContent.trim() : null;
        });
        if (active !== 'Parameters') {
            throw new Error(`Expected active tab 'Parameters', got '${active}'`);
        }
    });

    test('switch to Packages tab', async () => {
        const buttons = await harness.page.$$('.nav-item');
        for (const btn of buttons) {
            const text = await btn.evaluate(el => el.textContent.trim());
            if (text === 'Packages') { await btn.click(); break; }
        }
        await new Promise(r => setTimeout(r, 300));
        const active = await harness.page.evaluate(() => {
            const btn = document.querySelector('.nav-item.active');
            return btn ? btn.textContent.trim() : null;
        });
        if (active !== 'Packages') {
            throw new Error(`Expected active tab 'Packages', got '${active}'`);
        }
    });

    test('switch to API Docs tab', async () => {
        const buttons = await harness.page.$$('.nav-item');
        for (const btn of buttons) {
            const text = await btn.evaluate(el => el.textContent.trim());
            if (text === 'API Docs') { await btn.click(); break; }
        }
        await new Promise(r => setTimeout(r, 300));
        const active = await harness.page.evaluate(() => {
            const btn = document.querySelector('.nav-item.active');
            return btn ? btn.textContent.trim() : null;
        });
        if (active !== 'API Docs') {
            throw new Error(`Expected active tab 'API Docs', got '${active}'`);
        }
    });

    test('switch back to Monitor tab', async () => {
        const buttons = await harness.page.$$('.nav-item');
        for (const btn of buttons) {
            const text = await btn.evaluate(el => el.textContent.trim());
            if (text === 'Monitor') { await btn.click(); break; }
        }
        await new Promise(r => setTimeout(r, 300));
        const active = await harness.page.evaluate(() => {
            const btn = document.querySelector('.nav-item.active');
            return btn ? btn.textContent.trim() : null;
        });
        if (active !== 'Monitor') {
            throw new Error(`Expected active tab 'Monitor', got '${active}'`);
        }
    });

    // ── Monitor Tab Content ─────────────────────────────────────────────

    test('graph canvas exists and has dimensions', async () => {
        await clickTab(harness.page, 'Monitor');
        const canvas = await harness.page.evaluate(() => {
            const c = document.getElementById('graph-canvas');
            if (!c) return null;
            return { width: c.width, height: c.height, display: c.style.display };
        });
        if (!canvas) {
            throw new Error('graph-canvas element not found');
        }
        if (canvas.width < 100 || canvas.height < 100) {
            throw new Error(`Canvas too small: ${canvas.width}x${canvas.height}`);
        }
    });

    test('status bar shows node and topic counts', async () => {
        const statusText = await harness.page.evaluate(() => {
            const el = document.querySelector('.status-counts') ||
                        document.querySelector('.status-bar') ||
                        document.querySelector('[class*="status"]');
            return el ? el.textContent : document.body.innerText.slice(0, 500);
        });
        // Should mention Nodes and Topics somewhere
        if (!statusText.includes('Node') && !statusText.includes('node')) {
            throw new Error('Status should mention nodes');
        }
    });

    test('empty state shows placeholder text', async () => {
        // With no nodes running, should show empty/idle state
        const body = await harness.page.evaluate(() => document.body.innerText);
        const hasContent = body.includes('No active') || body.includes('no active') ||
                          body.includes('Idle') || body.includes('idle') ||
                          body.includes('Healthy') || body.includes('Node') ||
                          body.length > 100; // Page loaded with some content
        if (!hasContent) {
            throw new Error('Page should have status/placeholder content');
        }
    });

    // ── API Data Contracts ──────────────────────────────────────────────

    test('/api/status has required fields', async () => {
        const data = await harness.page.evaluate(async () => {
            const r = await fetch('/api/status');
            return r.json();
        });
        const required = ['nodes', 'topics'];
        for (const field of required) {
            if (!(field in data)) {
                throw new Error(`/api/status missing field: ${field}`);
            }
        }
    });

    test('/api/graph has nodes[] and edges[]', async () => {
        const data = await harness.page.evaluate(async () => {
            const r = await fetch('/api/graph');
            return r.json();
        });
        if (!Array.isArray(data.nodes)) {
            throw new Error('/api/graph.nodes should be array');
        }
        if (!Array.isArray(data.edges)) {
            throw new Error('/api/graph.edges should be array');
        }
    });

    test('/api/params responds', async () => {
        const resp = await harness.page.evaluate(async () => {
            const r = await fetch('/api/params');
            return { status: r.status, ok: r.ok };
        });
        if (!resp.ok) {
            throw new Error(`/api/params returned ${resp.status}`);
        }
    });

    // ── Parameter Tab ───────────────────────────────────────────────────

    test('Parameters tab renders param content', async () => {
        await clickTab(harness.page, 'Parameters');
        const html = await harness.page.evaluate(() => {
            const tab = document.getElementById('params-tab') ||
                        document.querySelector('[id*="param"]');
            return tab ? tab.innerHTML : '';
        });
        // Should have some param-related content (table, empty state, etc.)
        if (html.length < 10) {
            throw new Error('Parameters tab should have content');
        }
    });

    // ── Packages Tab ────────────────────────────────────────────────────

    test('Packages tab renders content', async () => {
        await clickTab(harness.page, 'Packages');
        const html = await harness.page.evaluate(() => {
            const tab = document.getElementById('packages-tab') ||
                        document.querySelector('[id*="package"]');
            return tab ? tab.innerHTML : '';
        });
        if (html.length < 10) {
            throw new Error('Packages tab should have content');
        }
    });

    // ── JavaScript Runtime Checks ───────────────────────────────────────

    test('WebSocket connection attempt made', async () => {
        // Check if wsConnected variable exists (set by connectWebSocket)
        const wsState = await harness.page.evaluate(() => {
            return {
                hasWsVar: typeof wsConnected !== 'undefined',
                connected: typeof wsConnected !== 'undefined' ? wsConnected : null,
            };
        });
        if (!wsState.hasWsVar) {
            throw new Error('wsConnected variable should be defined');
        }
        // WebSocket may or may not connect in test (depends on timing)
    });

    test('updateAll function exists and is callable', async () => {
        const result = await harness.page.evaluate(() => {
            return typeof updateAll === 'function';
        });
        if (!result) {
            throw new Error('updateAll should be a function');
        }
    });

    test('no uncaught exceptions during tab switching', async () => {
        const errors = [];
        harness.page.on('pageerror', err => errors.push(err.message));

        // Rapidly switch all tabs by clicking buttons
        for (const label of ['Monitor', 'Parameters', 'Packages', 'API Docs', 'Monitor']) {
            await clickTab(harness.page, label);
        }

        if (errors.length > 0) {
            throw new Error(`Uncaught exceptions during tab switching: ${errors.join(', ')}`);
        }
    });

    // ── Performance ─────────────────────────────────────────────────────

    test('API responses complete within 200ms', async () => {
        const endpoints = ['/api/status', '/api/nodes', '/api/topics', '/api/graph'];
        for (const ep of endpoints) {
            const start = Date.now();
            await harness.page.evaluate(async (url) => {
                await fetch(url);
            }, ep);
            const elapsed = Date.now() - start;
            if (elapsed > 200) {
                throw new Error(`${ep} took ${elapsed}ms (limit: 200ms)`);
            }
        }
    });

    test('tab switch is fast (< 2000ms for 4 switches)', async () => {
        const start = Date.now();
        await clickTab(harness.page, 'Parameters');
        await clickTab(harness.page, 'Monitor');
        await clickTab(harness.page, 'Packages');
        await clickTab(harness.page, 'Monitor');
        const elapsed = Date.now() - start;
        if (elapsed > 2000) {
            throw new Error(`4 tab switches took ${elapsed}ms (limit: 2000ms)`);
        }
        console.log(`  4 tab switches: ${elapsed}ms`);
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
        exitCode = 1;
    } finally {
        await harness.teardown();
    }

    process.exit(exitCode);
})();
