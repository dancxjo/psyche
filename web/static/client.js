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
    if (data.startsWith('ZENOH_INIT:')) {
        try {
            const obj = JSON.parse(data.slice('ZENOH_INIT:'.length));
            const keys = obj.keys || [];
            const status = obj.status || {};
            const info = document.getElementById('zenohInfo');
            const list = document.getElementById('zenohList');
            list.innerHTML = '';
            if (!keys.length) {
                info.textContent = 'No zenoh keys configured.';
            } else {
                info.textContent = 'Configured zenoh keys:';
                keys.forEach(k => {
                    const li = document.createElement('li');
                    li.id = 'zenoh-' + k.replace(/[^a-zA-Z0-9_\-]/g, '_');
                    const last = (status[k] && status[k].last) ? status[k].last : '(no data yet)';
                    li.innerHTML = `<strong>${k}</strong>: <span class="zen-last">${last}</span>`;
                    list.appendChild(li);
                });
            }
        } catch (e) {
            console.error('Invalid ZENOH_INIT payload', e);
        }
        return;
    }
    if (data.startsWith('ZENOH_UPD:')) {
        try {
            const obj = JSON.parse(data.slice('ZENOH_UPD:'.length));
            const key = obj.key;
            const payload = obj.payload;
            const safeId = 'zenoh-' + key.replace(/[^a-zA-Z0-9_\-]/g, '_');
            let li = document.getElementById(safeId);
            if (!li) {
                li = document.createElement('li');
                li.id = safeId;
                li.innerHTML = `<strong>${key}</strong>: <span class="zen-last">(no data yet)</span>`;
                document.getElementById('zenohList').appendChild(li);
            }
            const span = li.querySelector('.zen-last');
            if (span) span.textContent = payload;
            // also add to the general topics list for visibility
            addTopic(`ZENOH ${key}: ${payload}`);
        } catch (e) {
            console.error('Invalid ZENOH_UPD payload', e);
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
