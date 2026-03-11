"""
Simple Pythonic Async API for HORUS

Makes async I/O dead simple with Python's native async/await.
Just use 'async def tick()' and 'await' - that's it.
"""

import asyncio
from typing import Optional, Any
from . import Node


class AsyncNode(Node):
    """
    Simple async node - just use async/await!

    Example:
        ```python
        import horus

        class MyAsyncNode(horus.AsyncNode):
            async def async_init(self):
                print("Initialized!")

            async def async_tick(self):
                if self.has_msg("input"):
                    data = self.get("input")
                    result = await self.process(data)
                    self.send("output", result)

            async def process(self, msg):
                await asyncio.sleep(0.01)
                return msg * 2

        node = MyAsyncNode(
            name="async_processor",
            subs=["input"],
            pubs=["output"],
            rate=30,
        )
        horus.run(node)
        ```
    """

    def __init__(self, **kwargs):
        # Wire our sync bridge as the tick/init/shutdown callbacks
        kwargs.setdefault('tick', self._sync_tick)
        kwargs.setdefault('init', self._sync_init)
        kwargs.setdefault('shutdown', self._sync_shutdown)
        super().__init__(**kwargs)
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    async def async_init(self):
        """Override this for async one-time setup."""
        pass

    async def async_tick(self):
        """Override this with your async logic."""
        pass

    async def async_shutdown(self):
        """Override this for async cleanup."""
        pass

    def _get_loop(self) -> asyncio.AbstractEventLoop:
        """Get or create event loop."""
        if self._loop is None:
            try:
                self._loop = asyncio.get_running_loop()
            except RuntimeError:
                self._loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self._loop)
        return self._loop

    def _sync_init(self, node):
        """Bridge: called by scheduler, runs async_init."""
        loop = self._get_loop()
        loop.run_until_complete(self.async_init())

    def _sync_tick(self, node):
        """Bridge: called by scheduler, runs async_tick."""
        loop = self._get_loop()
        loop.run_until_complete(self.async_tick())

    def _sync_shutdown(self, node):
        """Bridge: called by scheduler, runs async_shutdown."""
        loop = self._get_loop()
        loop.run_until_complete(self.async_shutdown())


class AsyncTopic:
    """
    Async wrapper for Topic - use with await.

    Example:
        ```python
        from horus import AsyncTopic, CmdVel

        topic = AsyncTopic(CmdVel)

        # Send (async)
        await topic.send(CmdVel(linear=1.0, angular=0.5))

        # Receive (async, waits for message)
        msg = await topic.recv()

        # Try receive (async, returns None if no message)
        msg = await topic.try_recv()
        ```
    """

    def __init__(self, msg_type, capacity: int = 1024):
        from . import Topic
        self._topic = Topic(msg_type, capacity)

    async def send(self, msg: Any):
        """Send message asynchronously."""
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._topic.send, msg)

    async def recv(self, timeout: Optional[float] = None) -> Any:
        """
        Receive message asynchronously.
        Waits until a message is available, with exponential backoff.

        Args:
            timeout: Maximum seconds to wait (None = wait forever).

        Raises:
            asyncio.TimeoutError: If timeout expires before a message arrives.
        """
        loop = asyncio.get_event_loop()
        delay = 0.001  # Start at 1ms
        max_delay = 0.05  # Cap at 50ms
        elapsed = 0.0
        while True:
            msg = await loop.run_in_executor(None, self._topic.recv)
            if msg is not None:
                return msg
            if timeout is not None and elapsed >= timeout:
                raise asyncio.TimeoutError(
                    f"AsyncTopic.recv() timed out after {timeout}s"
                )
            await asyncio.sleep(delay)
            elapsed += delay
            delay = min(delay * 1.5, max_delay)

    async def try_recv(self) -> Optional[Any]:
        """
        Try to receive message asynchronously.
        Returns None immediately if no message.
        """
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self._topic.recv)


__all__ = [
    'AsyncNode',
    'AsyncTopic',
]
