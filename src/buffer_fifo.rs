/// A simple FIFO buffer implementation with a fixed-size array.
pub struct BufferFIFO<T, const N: usize> {
    buffer: [T; N],
    idx: usize,
    cap: usize,
}

impl<T, const N: usize> BufferFIFO<T, N>
where
    T: Default + Copy,
{
    /// Creates a new BufferFIFO with default values.
    pub fn default() -> Self {
        Self {
            buffer: [T::default(); N],
            cap: 0,
            idx: 0,
        }
    }

    /// Writes a value into the buffer, overwriting if full.
    pub fn write(&mut self, value: T) {
        self.buffer[self.idx] = value;
        self.idx = (self.idx + 1) % N;
        if self.cap < N {
            self.cap += 1;
        }
    }

    /// Reads (and removes) the oldest value in the buffer.
    pub fn read(&mut self) -> T {
        let val = self.buffer[(self.idx + N - self.cap) % N];
        if self.cap > 0 {
            self.cap -= 1;
        }
        val
    }

    /// Checks if the buffer is full.
    pub fn is_full(&self) -> bool {
        self.cap == N
    }

    /// Checks if the buffer is empty.
    pub fn is_empty(&self) -> bool {
        self.cap == 0
    }
}
