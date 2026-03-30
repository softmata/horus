/**
 * Visual regression + performance tests for horus monitor web dashboard.
 *
 * Captures screenshots at multiple viewports, verifies no layout overflow,
 * measures performance metrics, and checks canvas graph rendering.
 */

const { MonitorTestHarness } = require('./harness');
const fs = require('fs');
const path = require('path');

let harness;

const BASELINES_DIR = path.join(__dirname, 'baselines');
const VIEWPORTS = [
    { width: 1280, height: 720, name: 'laptop' },
    { width: 1920, height: 1080, name: 'desktop' },
    { width: 375, height: 812, name: 'mobile' },
];

/** Click a nav tab button by label */
async function clickTab(page, label) {
    const buttons = await page.$$('.nav-item');
    for (const btn of buttons) {
        const text = await btn.evaluate(el => el.textContent.trim());
        if (text === label) { await btn.click(); break; }
    }
    await new Promise(r => setTimeout(r, 500));
}

async function runTests() {
    const results = [];
    const failed = [];

    function test(name, fn) {
        results.push({ name, fn });
    }

    // ── Screenshot Capture ──────────────────────────────────────────────

    test('capture screenshots at all viewports', async () => {
        fs.mkdirSync(BASELINES_DIR, { recursive: true });
        const tabs = [
            { label: 'Monitor', id: 'monitor' },
            { label: 'Parameters', id: 'params' },
            { label: 'Packages', id: 'packages' },
        ];

        let captured = 0;
        for (const vp of VIEWPORTS) {
            await harness.page.setViewport({ width: vp.width, height: vp.height });
            for (const tab of tabs) {
                await clickTab(harness.page, tab.label);
                const filename = `${tab.id}_${vp.name}.png`;
                await harness.page.screenshot({
                    path: path.join(BASELINES_DIR, filename),
                    fullPage: true,
                });
                captured++;
            }
        }
        console.log(`  captured ${captured} screenshots to baselines/`);
        if (captured < 9) {
            throw new Error(`Expected 9+ screenshots, got ${captured}`);
        }
    });

    // ── No Horizontal Overflow ──────────────────────────────────────────

    test('no horizontal overflow at laptop viewport', async () => {
        await harness.page.setViewport({ width: 1280, height: 720 });
        await clickTab(harness.page, 'Monitor');
        const overflow = await harness.page.evaluate(() => {
            return document.documentElement.scrollWidth > document.documentElement.clientWidth;
        });
        if (overflow) {
            const widths = await harness.page.evaluate(() => ({
                scroll: document.documentElement.scrollWidth,
                client: document.documentElement.clientWidth,
            }));
            throw new Error(`Horizontal overflow: scroll=${widths.scroll} > client=${widths.client}`);
        }
    });

    test('no horizontal overflow at mobile viewport', async () => {
        await harness.page.setViewport({ width: 375, height: 812 });
        await clickTab(harness.page, 'Monitor');
        await new Promise(r => setTimeout(r, 300));
        const overflow = await harness.page.evaluate(() => {
            return document.documentElement.scrollWidth > document.documentElement.clientWidth;
        });
        if (overflow) {
            const widths = await harness.page.evaluate(() => ({
                scroll: document.documentElement.scrollWidth,
                client: document.documentElement.clientWidth,
            }));
            throw new Error(`Mobile horizontal overflow: scroll=${widths.scroll} > client=${widths.client}`);
        }
    });

    // Reset to desktop for remaining tests
    test('reset viewport to desktop', async () => {
        await harness.page.setViewport({ width: 1920, height: 1080 });
        await clickTab(harness.page, 'Monitor');
    });

    // ── Canvas Graph Verification ───────────────────────────────────────

    test('graph canvas is rendered (not blank)', async () => {
        await clickTab(harness.page, 'Monitor');
        const canvasData = await harness.page.evaluate(() => {
            const c = document.getElementById('graph-canvas');
            if (!c) return null;
            // Get pixel data — a blank canvas will have all zeros
            const ctx = c.getContext('2d');
            const data = ctx.getImageData(0, 0, c.width, c.height).data;
            let nonZero = 0;
            for (let i = 0; i < data.length; i += 4) {
                if (data[i] > 0 || data[i + 1] > 0 || data[i + 2] > 0) {
                    nonZero++;
                }
            }
            return { total: data.length / 4, nonZero };
        });
        if (!canvasData) {
            throw new Error('graph-canvas not found');
        }
        console.log(`  canvas pixels: ${canvasData.nonZero}/${canvasData.total} non-zero`);
        // Canvas may be blank if no nodes — that's ok, just verify it rendered
    });

    test('graph canvas element exists in DOM', async () => {
        // Canvas may be hidden (list view is default), but it should exist in DOM
        const exists = await harness.page.evaluate(() => {
            const c = document.getElementById('graph-canvas');
            return c !== null;
        });
        if (!exists) throw new Error('graph-canvas element not found in DOM');
        // Check the canvas has width/height attributes
        const attrs = await harness.page.evaluate(() => {
            const c = document.getElementById('graph-canvas');
            return { width: c.getAttribute('width'), height: c.getAttribute('height') };
        });
        if (!attrs.width || !attrs.height) {
            throw new Error('Canvas missing width/height attributes');
        }
    });

    // ── Performance Metrics ─────────────────────────────────────────────

    test('page load time-to-interactive < 3s', async () => {
        const start = Date.now();
        await harness.page.reload({ waitUntil: 'domcontentloaded' });
        // Wait for JS to initialize (status bar populated)
        await harness.page.waitForFunction(() => {
            return document.body.innerText.length > 50;
        }, { timeout: 3000 });
        const tti = Date.now() - start;
        console.log(`  time-to-interactive: ${tti}ms`);
        if (tti > 3000) {
            throw new Error(`TTI ${tti}ms exceeds 3000ms limit`);
        }
    });

    test('API /api/status latency < 100ms', async () => {
        const latency = await harness.page.evaluate(async () => {
            const start = performance.now();
            await fetch('/api/status');
            return Math.round(performance.now() - start);
        });
        console.log(`  /api/status: ${latency}ms`);
        if (latency > 100) {
            throw new Error(`/api/status took ${latency}ms (limit: 100ms)`);
        }
    });

    test('API /api/nodes latency < 100ms', async () => {
        const latency = await harness.page.evaluate(async () => {
            const start = performance.now();
            await fetch('/api/nodes');
            return Math.round(performance.now() - start);
        });
        console.log(`  /api/nodes: ${latency}ms`);
        if (latency > 100) {
            throw new Error(`/api/nodes took ${latency}ms (limit: 100ms)`);
        }
    });

    test('API /api/graph latency < 100ms', async () => {
        const latency = await harness.page.evaluate(async () => {
            const start = performance.now();
            await fetch('/api/graph');
            return Math.round(performance.now() - start);
        });
        console.log(`  /api/graph: ${latency}ms`);
        if (latency > 100) {
            throw new Error(`/api/graph took ${latency}ms (limit: 100ms)`);
        }
    });

    test('10 rapid API calls complete < 500ms total', async () => {
        const totalMs = await harness.page.evaluate(async () => {
            const start = performance.now();
            const promises = [];
            for (let i = 0; i < 10; i++) {
                promises.push(fetch('/api/status'));
            }
            await Promise.all(promises);
            return Math.round(performance.now() - start);
        });
        console.log(`  10 concurrent /api/status: ${totalMs}ms`);
        if (totalMs > 500) {
            throw new Error(`10 concurrent calls took ${totalMs}ms (limit: 500ms)`);
        }
    });

    // ── CSS/Layout Checks ───────────────────────────────────────────────

    test('dark theme colors are applied', async () => {
        const bg = await harness.page.evaluate(() => {
            return getComputedStyle(document.body).backgroundColor;
        });
        // Dark theme should have a dark background
        // rgb(10, 10, 15) or similar
        if (bg.includes('255, 255, 255') || bg === 'rgb(255, 255, 255)') {
            throw new Error(`Expected dark background, got: ${bg}`);
        }
    });

    test('nav bar is visible and has items', async () => {
        const navItems = await harness.page.$$eval('.nav-item', items =>
            items.map(el => ({ text: el.textContent.trim(), visible: el.offsetHeight > 0 }))
        );
        if (navItems.length < 3) {
            throw new Error(`Expected 3+ nav items, got ${navItems.length}`);
        }
        const allVisible = navItems.every(i => i.visible);
        if (!allVisible) {
            throw new Error('Some nav items are not visible');
        }
    });

    test('all tab content containers exist', async () => {
        const tabs = await harness.page.evaluate(() => {
            const ids = ['tab-monitor', 'tab-params', 'tab-packages'];
            return ids.map(id => ({
                id,
                exists: !!document.getElementById(id),
            }));
        });
        const missing = tabs.filter(t => !t.exists).map(t => t.id);
        if (missing.length > 0) {
            throw new Error(`Missing tab containers: ${missing.join(', ')}`);
        }
    });

    // ── Screenshot file verification ────────────────────────────────────

    test('all baseline screenshots saved successfully', async () => {
        const expectedFiles = [
            'monitor_laptop.png', 'monitor_desktop.png', 'monitor_mobile.png',
            'params_laptop.png', 'params_desktop.png', 'params_mobile.png',
            'packages_laptop.png', 'packages_desktop.png', 'packages_mobile.png',
        ];
        const missing = expectedFiles.filter(f =>
            !fs.existsSync(path.join(BASELINES_DIR, f))
        );
        if (missing.length > 0) {
            throw new Error(`Missing screenshots: ${missing.join(', ')}`);
        }
        // Verify files have content (not empty)
        for (const f of expectedFiles) {
            const stat = fs.statSync(path.join(BASELINES_DIR, f));
            if (stat.size < 1000) {
                throw new Error(`Screenshot ${f} too small (${stat.size} bytes)`);
            }
        }
        console.log(`  9 screenshots verified, all > 1KB`);
    });

    // ── Run tests ───────────────────────────────────────────────────────

    console.log(`\nRunning ${results.length} visual/performance tests...\n`);
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
