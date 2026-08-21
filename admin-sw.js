const CACHE_VERSION = "kimo-admin-pwa-v3";
const APP_HOME = "/mobile-admin.html";
const OFFLINE_PAGE = "/offline-admin.html";

const APP_SHELL = [
  APP_HOME,
  OFFLINE_PAGE,
  "/mobile-admin.css",
  "/mobile-admin-data.js",
  "/admin-pwa.css",
  "/admin-pwa.js",
  "/admin.webmanifest",
  "/favicon.png",
  "/logo.png",
  "/images/pwa/admin-icon-192.png",
  "/images/pwa/admin-icon-512.png",
  "/images/pwa/admin-maskable-512.png",
  "/images/pwa/apple-touch-icon-180.png"
];

const CACHEABLE_PATHS = new Set(APP_SHELL);

self.addEventListener("install", (event) => {
  event.waitUntil(
    caches.open(CACHE_VERSION)
      .then((cache) => cache.addAll(APP_SHELL))
      .then(() => self.skipWaiting())
  );
});

self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches.keys()
      .then((keys) => Promise.all(
        keys
          .filter((key) => key.startsWith("kimo-admin-pwa-") && key !== CACHE_VERSION)
          .map((key) => caches.delete(key))
      ))
      .then(() => self.clients.claim())
  );
});

self.addEventListener("message", (event) => {
  if(event.data?.type === "SKIP_WAITING"){
    self.skipWaiting();
  }
});

self.addEventListener("fetch", (event) => {
  const request = event.request;
  if(request.method !== "GET") return;

  const url = new URL(request.url);
  if(url.origin !== self.location.origin) return;

  if(request.mode === "navigate"){
    event.respondWith(handleNavigation(request, url));
    return;
  }

  if(CACHEABLE_PATHS.has(url.pathname)){
    event.respondWith(cacheFirst(request));
  }
});

async function handleNavigation(request, url){
  try{
    return await fetch(request);
  }catch(error){
    const cache = await caches.open(CACHE_VERSION);

    if(url.pathname === APP_HOME || url.pathname === "/"){
      return (await cache.match(APP_HOME)) || (await cache.match(OFFLINE_PAGE));
    }

    return (await cache.match(OFFLINE_PAGE)) || Response.error();
  }
}

async function cacheFirst(request){
  const cache = await caches.open(CACHE_VERSION);
  const cached = await cache.match(request, { ignoreSearch:true });
  if(cached) return cached;

  const response = await fetch(request);
  if(response.ok) cache.put(request, response.clone());
  return response;
}
