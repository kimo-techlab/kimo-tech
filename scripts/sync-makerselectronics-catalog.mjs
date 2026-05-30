import { mkdir, writeFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const SOURCE_BASE_URL = "https://makerselectronics.com/wp-json/wc/store/v1/products";
const PAGE_SIZE = 100;
const CONCURRENCY = 3;
const OUTPUT_PATH = path.resolve(__dirname, "..", "data", "makers-electronics-products.json");
const OUTPUT_JS_PATH = path.resolve(__dirname, "..", "data", "makers-electronics-products.js");
const EMBEDDED_GLOBAL_NAME = "__MAKERS_ELECTRONICS_PRODUCTS__";

const NAMED_ENTITIES = {
  amp: "&",
  apos: "'",
  gt: ">",
  lt: "<",
  nbsp: " ",
  ndash: "-",
  mdash: "-",
  quot: "\""
};

function decodeHtmlEntities(value) {
  return String(value ?? "").replace(/&(#x?[0-9a-f]+|[a-z]+);/gi, (match, entity) => {
    const normalized = String(entity).toLowerCase();

    if (normalized.startsWith("#x")) {
      const codePoint = Number.parseInt(normalized.slice(2), 16);
      return Number.isFinite(codePoint) ? String.fromCodePoint(codePoint) : match;
    }

    if (normalized.startsWith("#")) {
      const codePoint = Number.parseInt(normalized.slice(1), 10);
      return Number.isFinite(codePoint) ? String.fromCodePoint(codePoint) : match;
    }

    return NAMED_ENTITIES[normalized] ?? match;
  });
}

function stripHtml(html) {
  const withoutTags = String(html ?? "")
    .replace(/<style[\s\S]*?<\/style>/gi, " ")
    .replace(/<script[\s\S]*?<\/script>/gi, " ")
    .replace(/<[^>]+>/g, " ");

  return decodeHtmlEntities(withoutTags)
    .replace(/\s+/g, " ")
    .trim();
}

function clampText(value, maxLength) {
  const text = String(value ?? "").trim();

  if (!text || text.length <= maxLength) {
    return text;
  }

  return `${text.slice(0, Math.max(0, maxLength - 1)).trim()}…`;
}

function normalizeCategory(categories) {
  if (!Array.isArray(categories) || categories.length === 0) {
    return "Other";
  }

  const firstNamed = categories.find((category) => String(category?.name ?? "").trim());
  return decodeHtmlEntities(firstNamed?.name ?? "Other").trim() || "Other";
}

function mapProduct(product, sequence) {
  const minorUnit = Number(product?.prices?.currency_minor_unit ?? 2);
  const rawPrice = Number(product?.prices?.price ?? 0);
  const priceDivisor = Number.isFinite(minorUnit) ? 10 ** minorUnit : 100;
  const price = Number.isFinite(rawPrice) ? rawPrice / priceDivisor : 0;
  const images = Array.isArray(product?.images) ? product.images : [];
  const primaryImage = images[0] ?? {};
  const shortText = stripHtml(product?.short_description);
  const longText = stripHtml(product?.description);
  const description = clampText(shortText || longText, 360);

  return {
    id: `makers-${product.id}`,
    sourceSite: "makerselectronics.com",
    sourceProductId: Number(product.id),
    sourceUrl: product?.permalink ?? "",
    sku: String(product?.sku ?? ""),
    name: decodeHtmlEntities(product?.name ?? "").trim(),
    price: Number.isFinite(price) ? price : 0,
    category: normalizeCategory(product?.categories),
    image: primaryImage.thumbnail || primaryImage.src || "images/no-image.png",
    fullImage: primaryImage.src || primaryImage.thumbnail || "images/no-image.png",
    use: description,
    description,
    badge: product?.on_sale ? "Sale" : "",
    inStock: Boolean(product?.is_in_stock),
    stockQty: product?.low_stock_remaining ?? null,
    order: 100000 + sequence
  };
}

async function fetchJson(url) {
  const maxAttempts = 5;

  for (let attempt = 1; attempt <= maxAttempts; attempt += 1) {
    const response = await fetch(url, {
      headers: {
        "User-Agent": "KimoTechCatalogSync/1.0"
      }
    });

    if (response.ok) {
      return {
        headers: response.headers,
        json: await response.json()
      };
    }

    const shouldRetry = [403, 408, 429, 500, 502, 503, 504].includes(response.status);

    if (!shouldRetry || attempt === maxAttempts) {
      throw new Error(`Failed to fetch ${url}: HTTP ${response.status}`);
    }

    const backoffMs = attempt * 1500;
    console.warn(`Retrying ${url} after HTTP ${response.status} (attempt ${attempt}/${maxAttempts})`);
    await new Promise((resolve) => setTimeout(resolve, backoffMs));
  }
}

async function fetchPage(pageNumber) {
  const url = `${SOURCE_BASE_URL}?per_page=${PAGE_SIZE}&page=${pageNumber}`;
  const { json } = await fetchJson(url);
  return Array.isArray(json) ? json : [];
}

async function runWorker(queue, results) {
  while (queue.length) {
    const pageNumber = queue.shift();

    if (pageNumber == null) {
      return;
    }

    const items = await fetchPage(pageNumber);
    results.set(pageNumber, items);
    console.log(`Fetched page ${pageNumber} (${items.length} products)`);
  }
}

async function main() {
  const firstUrl = `${SOURCE_BASE_URL}?per_page=${PAGE_SIZE}&page=1`;
  const firstResponse = await fetchJson(firstUrl);
  const totalProducts = Number(firstResponse.headers.get("x-wp-total") ?? 0);
  const totalPages = Number(firstResponse.headers.get("x-wp-totalpages") ?? 1);
  const pageResults = new Map([[1, Array.isArray(firstResponse.json) ? firstResponse.json : []]]);

  console.log(`Source reports ${totalProducts} products across ${totalPages} pages.`);

  const remainingPages = Array.from({ length: Math.max(0, totalPages - 1) }, (_, index) => index + 2);
  const workers = Array.from(
    { length: Math.min(CONCURRENCY, remainingPages.length) },
    () => runWorker(remainingPages, pageResults)
  );

  await Promise.all(workers);

  const orderedPages = Array.from({ length: totalPages }, (_, index) => index + 1)
    .flatMap((pageNumber) => pageResults.get(pageNumber) ?? []);

  const products = orderedPages
    .map((product, index) => mapProduct(product, index))
    .filter((product) => product.name);

  const payload = {
    source: "makerselectronics.com",
    generatedAt: new Date().toISOString(),
    totalProducts: products.length,
    currency: "EGP",
    products
  };

  await mkdir(path.dirname(OUTPUT_PATH), { recursive: true });
  await writeFile(OUTPUT_PATH, JSON.stringify(payload, null, 2), "utf8");
  await writeFile(
    OUTPUT_JS_PATH,
    `globalThis.${EMBEDDED_GLOBAL_NAME} = ${JSON.stringify(payload, null, 2)};\n`,
    "utf8"
  );

  console.log(`Saved ${products.length} products to ${OUTPUT_PATH}`);
  console.log(`Saved embedded catalog to ${OUTPUT_JS_PATH}`);
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
