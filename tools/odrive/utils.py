#!/usr/bin/env python3
"""
Liveplotter
"""

import sys
import time
import threading

data_rate = 100
plot_rate = 10
num_samples = 1000

def start_liveplotter(get_var_callback):
    """
    Starts a liveplotter.
    The variable that is plotted is retrieved from get_var_callback.
    This function returns immediately and the liveplotter quits when
    the user closes it.
    """

    import matplotlib.pyplot as plt

    cancellation_token = threading.Event()

    global vals
    vals = []
    def fetch_data():
        global vals
        while not cancellation_token.is_set():
            try:
                data = get_var_callback()
            except Exception as ex:
                print(str(ex))
                time.sleep(1)
                continue
            vals.append(data)
            if len(vals) > num_samples:
                vals = vals[-num_samples:]
            time.sleep(1/data_rate)

    # TODO: use animation for better UI performance, see:
    # https://matplotlib.org/examples/animation/simple_anim.html
    def plot_data():
        global vals

        plt.ion()

        # Make sure the script terminates when the user closes the plotter
        def did_close(evt):
            cancellation_token.set()
        fig = plt.figure()
        fig.canvas.mpl_connect('close_event', did_close)

        while not cancellation_token.is_set():
            plt.clf()
            plt.plot(vals)
            #time.sleep(1/plot_rate)
            fig.canvas.flush_events()

    threading.Thread(target=fetch_data).start()
    threading.Thread(target=plot_data).start()
    #plot_data()

## Exceptions ##

class TimeoutException(Exception):
    pass

## Threading utils ##

class Event():
    """
    Alternative to threading.Event(), enhanced by the subscribe() function
    that the original fails to provide.
    """
    def __init__(self):
        self._evt = threading.Event()
        self._subscribers = []
        self._mutex = threading.Lock()

    def is_set(self):
        return self._evt.is_set()

    def set(self):
        """
        Sets the event and invokes all subscribers if the event was
        not already set
        """
        self._mutex.acquire()
        try:
            if not self._evt.is_set():
                self._evt.set()
                for s in self._subscribers:
                    s()
        finally:
            self._mutex.release()

    def subscribe(self, handler):
        """
        Invokes the specified handler exactly once as soon as the
        specified event is set. If the event is already set, the
        handler is invoked immediately.
        Returns a function that can be invoked to unsubscribe.
        """
        self._mutex.acquire()
        try:
            self._subscribers.append(handler)
            if self._evt.is_set():
                handler()
        finally:
            self._mutex.release()
        return lambda: self.unsubscribe(handler)
    
    def unsubscribe(self, handler):
        self._mutex.acquire()
        try:
            self._subscribers.pop(self._subscribers.index(handler))
        finally:
            self._mutex.release()

    def wait(self, timeout=None):
        return self._evt.wait(timeout=timeout)

def wait_any(*events, timeout=None):
    """
    Blocks until any of the specified events are triggered.
    Returns the number of the event that was triggerd or raises
    a TimeoutException
    """
    or_event = threading.Event()
    unsubscribe_functions = []
    for event in events:
        unsubscribe_functions.append(event.subscribe(lambda: or_event.set()))
    or_event.wait(timeout=timeout)
    for unsubscribe_function in unsubscribe_functions:
        unsubscribe_function()
    for i in range(len(events)):
        if events[i].is_set():
            return i
    raise TimeoutException()
