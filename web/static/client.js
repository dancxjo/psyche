const statusEl = document.getElementById('status');
const topicsEl = document.getElementById('topics');
const form = document.getElementById('addForm');
const input = document.getElementById('topicInput');

const wsProto = (location.protocol === 'https:') ? 'wss' : 'ws';
const wsUrl = wsProto + '://' + location.host + '/ws';
const socket = new WebSocket(wsUrl);

socket.addEventListener('open', () => {
    statusEl.textContent = 'Connected';
});

socket.addEventListener('message', (ev) => {
    const data = ev.data;
    if (data.startsWith('INIT:')) {
        const payload = data.slice(5);
        if (payload.length) {
            payload.split('||').forEach(addTopic);
        }
        return;
    }
    if (data.startsWith('ADD:')) {
        addTopic(data.slice(4));
        return;
    }
    // fallback: show raw
    addTopic(data);
});

socket.addEventListener('close', () => {
    statusEl.textContent = 'Disconnected';
});

function addTopic(text) {
    if (!text) return;
    const li = document.createElement('li');
    li.textContent = text;
    topicsEl.insertBefore(li, topicsEl.firstChild);
}

form.addEventListener('submit', (e) => {
    e.preventDefault();
    if (!input.value) return;
    // post to HTTP endpoint to add/broadcast
    fetch('/update', { method: 'POST', body: new URLSearchParams({ topic: input.value }) })
        .then(() => { input.value = ''; })
        .catch(err => console.error(err));
});
