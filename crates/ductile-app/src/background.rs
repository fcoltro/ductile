// SPDX-License-Identifier: GPL-3.0-only
//! Runs a tool's heavy computation off the UI thread, so dragging stays
//! smooth: the latest finished result is shown while the newest request
//! computes, and requests made meanwhile collapse into one.

use std::cell::RefCell;
use std::sync::mpsc::{channel, Receiver, TryRecvError};

type Job<V> = Box<dyn FnOnce() -> V + Send>;

struct State<K, V> {
    done: Option<(K, V)>,
    running: Option<(K, Receiver<V>)>,
    queued: Option<(K, Job<V>)>,
}

pub struct Background<K, V> {
    state: RefCell<State<K, V>>,
}

impl<K: PartialEq + Clone, V: Clone + Send + 'static> Default for Background<K, V> {
    fn default() -> Self {
        Self { state: RefCell::new(State { done: None, running: None, queued: None }) }
    }
}

impl<K: PartialEq + Clone, V: Clone + Send + 'static> Background<K, V> {
    /// The result for `key` if it is ready, else the latest one finished
    /// (for an older key), starting `make()`'s job when nothing computes
    /// `key` yet.
    pub fn get(&self, key: &K, make: impl FnOnce() -> Job<V>) -> Option<V> {
        let mut s = self.state.borrow_mut();
        Self::poll(&mut s);
        if let Some((k, v)) = &s.done {
            if k == key {
                return Some(v.clone());
            }
        }
        let pending = s.running.as_ref().map(|r| &r.0 == key).unwrap_or(false) || s.queued.as_ref().map(|q| &q.0 == key).unwrap_or(false);
        if !pending {
            if s.running.is_some() {
                s.queued = Some((key.clone(), make()));
            } else {
                s.running = Some((key.clone(), spawn(make())));
            }
        }
        s.done.as_ref().map(|d| d.1.clone())
    }

    /// The result for `key`, waiting for (or computing) it if need be.
    pub fn wait(&self, key: &K, make: impl FnOnce() -> Job<V>) -> V {
        let mut s = self.state.borrow_mut();
        Self::poll(&mut s);
        if let Some((k, v)) = &s.done {
            if k == key {
                return v.clone();
            }
        }
        if let Some((k, rx)) = s.running.take() {
            if &k == key {
                if let Ok(v) = rx.recv() {
                    s.done = Some((k, v.clone()));
                    return v;
                }
            }
        }
        s.queued = None;
        let v = make()();
        s.done = Some((key.clone(), v.clone()));
        v
    }

    /// The latest finished result, whatever its key.
    pub fn latest(&self) -> Option<V> {
        let mut s = self.state.borrow_mut();
        Self::poll(&mut s);
        s.done.as_ref().map(|d| d.1.clone())
    }

    /// Whether a result is still being computed.
    pub fn busy(&self) -> bool {
        let mut s = self.state.borrow_mut();
        Self::poll(&mut s);
        s.running.is_some()
    }

    fn poll(s: &mut State<K, V>) {
        if let Some((_, rx)) = &s.running {
            match rx.try_recv() {
                Ok(v) => {
                    let (k, _) = s.running.take().unwrap();
                    s.done = Some((k, v));
                }
                Err(TryRecvError::Disconnected) => s.running = None,
                Err(TryRecvError::Empty) => return,
            }
        }
        if let Some((k, job)) = s.queued.take() {
            s.running = Some((k, spawn(job)));
        }
    }
}

fn spawn<V: Send + 'static>(job: Job<V>) -> Receiver<V> {
    let (tx, rx) = channel();
    std::thread::spawn(move || {
        let _ = tx.send(job());
    });
    rx
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shows_the_last_result_while_the_next_computes() {
        let bg: Background<u32, u32> = Background::default();
        assert_eq!(bg.wait(&1, || Box::new(|| 10)), 10);
        let (tx, rx) = channel::<()>();
        let rx = std::sync::Mutex::new(rx);
        // Key 2 blocks until released; meanwhile key 1's result stands.
        let slow = move || -> Job<u32> {
            Box::new(move || {
                rx.lock().unwrap().recv().unwrap();
                20
            })
        };
        assert_eq!(bg.get(&2, slow), Some(10));
        assert!(bg.busy());
        tx.send(()).unwrap();
        assert_eq!(bg.wait(&2, || unreachable!()), 20);
        assert!(!bg.busy());
    }
}
