// Vercel Serverless API Endpoint with Self-Healing Cloud Persistence
// Path: api/journal.js

let currentBlobUrl = 'https://jsonblob.com/api/jsonBlob/019fb297-c286-7609-8165-90d10a10452c';
let memoryEntriesCache = null;

async function getOrCreateBlobUrl() {
  if (currentBlobUrl) {
    try {
      const checkRes = await fetch(currentBlobUrl);
      if (checkRes.ok) return currentBlobUrl;
    } catch (e) {}
  }

  // Create new blob if 404 or expired
  try {
    const createRes = await fetch('https://jsonblob.com/api/jsonBlob', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(memoryEntriesCache || [])
    });
    if (createRes.ok) {
      const loc = createRes.headers.get('Location');
      if (loc) {
        currentBlobUrl = loc.startsWith('http') ? loc : `https://jsonblob.com${loc}`;
        return currentBlobUrl;
      }
    }
  } catch (e) {}

  return currentBlobUrl;
}

export default async function handler(req, res) {
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type');

  if (req.method === 'OPTIONS') {
    return res.status(200).end();
  }

  const blobUrl = await getOrCreateBlobUrl();

  if (req.method === 'GET') {
    try {
      if (blobUrl) {
        const dbRes = await fetch(blobUrl);
        if (dbRes.ok) {
          const data = await dbRes.json();
          if (Array.isArray(data) && data.length > 0) {
            memoryEntriesCache = data;
            return res.status(200).json(data);
          }
        }
      }
    } catch (e) {}

    if (memoryEntriesCache && memoryEntriesCache.length > 0) {
      return res.status(200).json(memoryEntriesCache);
    }

    return res.status(200).json([]);
  }

  if (req.method === 'POST' || req.method === 'PUT') {
    const bodyData = req.body;
    let payload = null;

    if (Array.isArray(bodyData)) {
      payload = bodyData;
    } else if (bodyData && Array.isArray(bodyData.entries)) {
      payload = bodyData.entries;
    }

    if (!payload) {
      return res.status(400).json({ error: 'Missing or invalid entries array' });
    }

    memoryEntriesCache = payload;

    try {
      if (blobUrl) {
        await fetch(blobUrl, {
          method: 'PUT',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload)
        });
      }
    } catch (err) {}

    return res.status(200).json({ status: 'saved', count: payload.length });
  }

  res.status(405).end();
}
