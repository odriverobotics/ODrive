import io from 'socket.io-client';

let socket = undefined;
let url = 'https://0.0.0.0:8080';

function initSocket(url) {
    socket = io(url);
}

export function setUrl(path) {
    if (socket) {
        socket.close();
        socket = undefined;
    }
    url = path;
    initSocket(url);
}

export function closeSocket() {
    socket.close();
    socket = undefined;
}

export function addEventListener(event) {
    if (!socket) {
        initSocket(url);
    }
    socket.on(event.type, event.callback);
}

export function sendEvent(event) {
    socket.emit(event.type, event.data);
}