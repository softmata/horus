# Simple Async I/O in HORUS

Dead simple async I/O with Python's `async/await`. That's it.

## Quick Start

Just inherit from `AsyncNode` and use `async def tick()`:

```python
import horus

class MyNode(horus.AsyncNode):
    async def setup(self):
        self.sub = horus.AsyncTopic("input", str)
        self.pub = horus.AsyncTopic("output", str)

    async def tick(self):
        msg = await self.sub.recv()  # Non-blocking!
        result = await self.process(msg)
        await self.pub.send(result)

    async def process(self, msg):
        await horus.sleep(0.1)  # Non-blocking sleep
        return msg.upper()
```

That's the entire async API. No complex helpers, no thread pools, just `async/await`.

## Real-World Example: Database Queries

```python
import horus
import asyncpg  # pip install asyncpg

class DatabaseNode(horus.AsyncNode):
    async def setup(self):
        # Connect to PostgreSQL
        self.db = await asyncpg.connect('postgresql://localhost/mydb')

        self.query_sub = horus.AsyncTopic("query", str)
        self.result_pub = horus.AsyncTopic("result", str)

    async def tick(self):
        # Get query
        query = await self.query_sub.recv()

        # Execute (non-blocking!)
        rows = await self.db.fetch(query)

        # Send results
        await self.result_pub.send(str(rows))

    async def shutdown(self):
        await self.db.close()
```

## Real-World Example: HTTP APIs

```python
import horus
import aiohttp  # pip install aiohttp

class WeatherNode(horus.AsyncNode):
    async def setup(self):
        self.session = aiohttp.ClientSession()
        self.location_sub = horus.AsyncTopic("location", str)
        self.weather_pub = horus.AsyncTopic("weather", dict)

    async def tick(self):
        location = await self.location_sub.recv()

        # Fetch from API (non-blocking!)
        async with self.session.get(f'https://api.weather.com/{location}') as resp:
            data = await resp.json()

        await self.weather_pub.send(data)

    async def shutdown(self):
        await self.session.close()
```

## Concurrent Operations

Use Python's built-in `asyncio`:

```python
import horus

class SensorFusionNode(horus.AsyncNode):
    async def tick(self):
        # Fetch from multiple sensors concurrently
        camera_data, lidar_data, imu_data = await horus.gather(
            self.camera_sub.recv(),
            self.lidar_sub.recv(),
            self.imu_sub.recv()
        )

        # Fuse data
        result = self.fuse(camera_data, lidar_data, imu_data)

        await self.output_pub.send(result)
```

## Timeouts

```python
import horus

class SafeNode(horus.AsyncNode):
    async def tick(self):
        try:
            # Wait max 1 second for message
            msg = await horus.wait_for(
                self.sub.recv(),
                timeout=1.0
            )
        except asyncio.TimeoutError:
            print("No message received in 1 second")
```

## Use Any Async Library

HORUS async nodes work with **any** Python async library:

- **aiohttp**: HTTP clients/servers
- **asyncpg**: PostgreSQL
- **motor**: MongoDB
- **aioredis**: Redis
- **asyncio**: File I/O, subprocesses, streams

Just use them normally - no special HORUS integration needed!

## Complete API

That's the entire async API:

```python
import horus

# Classes
horus.AsyncNode      # Base class for async nodes
horus.AsyncTopic       # Async wrapper for Hub

# Functions
await horus.sleep(seconds)           # Non-blocking sleep
await horus.gather(*tasks)           # Run tasks concurrently
await horus.wait_for(task, timeout)  # Wait with timeout
```

**Three lines. That's it.**

Everything else is just standard Python `async/await`.

## Implementation Phases

All three phases are now complete with this simple design:

- **Phase 1**: Threading patterns - Use Python's threading if needed
- **Phase 2**: Async utilities - Use Python's asyncio + any async library
- **Phase 3**: Full async/await - `AsyncNode` + `AsyncTopic`

No complex helpers. No custom patterns. Just Pythonic async/await.
