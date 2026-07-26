import { initializeApp } from "https://www.gstatic.com/firebasejs/12.9.0/firebase-app.js";
import {
  getAuth,
  onAuthStateChanged,
  signOut
} from "https://www.gstatic.com/firebasejs/12.9.0/firebase-auth.js";
import {
  getFirestore,
  collection,
  doc,
  getDoc,
  limit,
  onSnapshot,
  query,
  updateDoc,
  where
} from "https://www.gstatic.com/firebasejs/12.9.0/firebase-firestore.js";

const firebaseConfig = {
  apiKey: "AIzaSyA2ff2ZaA-Xb_zQcVSUDaERS-glaFFR0TY",
  authDomain: "kimo-tech-store-962df.firebaseapp.com",
  projectId: "kimo-tech-store-962df",
  storageBucket: "kimo-tech-store-962df.firebasestorage.app",
  messagingSenderId: "845478890277",
  appId: "1:845478890277:web:3cdd3328bde1b5b9dda8cc"
};

const app = initializeApp(firebaseConfig);
const auth = getAuth(app);
const db = getFirestore(app);
const allowedRoles = new Set(["owner", "admin"]);
const countFormatter = new Intl.NumberFormat("ar-EG");
const moneyFormatter = new Intl.NumberFormat("ar-EG", { maximumFractionDigits:2 });
const dateFormatter = new Intl.DateTimeFormat("ar-EG", {
  dateStyle:"medium",
  timeStyle:"short"
});

const recentOrdersContainer = document.getElementById("recentOrders");
const recentInvoicesContainer = document.getElementById("recentInvoices");
const lowStockContainer = document.getElementById("lowStockProducts");
const dashboardContent = document.getElementById("mobileDashboardContent");
const loadingCard = document.getElementById("mobileAppLoading");
const appStatus = document.getElementById("appDataStatus");
const syncTime = document.getElementById("syncTime");
const logoutButton = document.getElementById("logoutAppButton");
const mobileToast = document.getElementById("mobileAdminToast");

const loadStates = {
  orders:"loading",
  invoices:"loading",
  products:"idle"
};

let currentRole = "";
let allOrders = [];
let allInvoices = [];
let lowStockProducts = [];
let unsubscribeListeners = [];
let toastTimer = 0;
let redirectingToLogin = false;

function showToast(message, type = "success", duration = 3800){
  clearTimeout(toastTimer);
  mobileToast.hidden = false;
  mobileToast.className = `mobile-admin-toast ${type}`;
  mobileToast.setAttribute("role", type === "error" ? "alert" : "status");
  mobileToast.textContent = message;

  toastTimer = window.setTimeout(() => {
    mobileToast.hidden = true;
  }, duration);
}

window.showMobileAdminToast = showToast;

function redirectToLogin(message = ""){
  if(redirectingToLogin) return;
  redirectingToLogin = true;

  if(message) sessionStorage.setItem("adminAuthMessage", message);
  sessionStorage.removeItem("adminRole");
  window.location.replace("admin-login.html?next=mobile-admin.html");
}

function getSafeDate(value){
  if(!value) return null;

  try{
    if(value instanceof Date) return Number.isNaN(value.getTime()) ? null : value;
    if(typeof value.toDate === "function"){
      const date = value.toDate();
      return Number.isNaN(date.getTime()) ? null : date;
    }
    if(typeof value.seconds === "number"){
      const date = new Date(value.seconds * 1000);
      return Number.isNaN(date.getTime()) ? null : date;
    }
    if(typeof value === "string" || typeof value === "number"){
      const date = new Date(value);
      return Number.isNaN(date.getTime()) ? null : date;
    }
  }catch(error){
    console.warn("Invalid date value:", error);
  }

  return null;
}

function toEnglishDigits(value){
  const arabicDigits = "٠١٢٣٤٥٦٧٨٩";
  const persianDigits = "۰۱۲۳۴۵۶۷۸۹";

  return String(value ?? "")
    .replace(/[٠-٩]/g, (digit) => String(arabicDigits.indexOf(digit)))
    .replace(/[۰-۹]/g, (digit) => String(persianDigits.indexOf(digit)));
}

function parseInvoiceDate(value){
  const normalized = toEnglishDigits(value)
    .replace(/[\u061C\u200E\u200F\u202A-\u202E\u2066-\u2069]/g, "")
    .trim();
  const match = normalized.match(/^(\d{1,2})[\/.\-](\d{1,2})[\/.\-](\d{4})$/);

  if(match){
    const day = Number(match[1]);
    const month = Number(match[2]);
    const year = Number(match[3]);
    const date = new Date(year, month - 1, day);

    if(
      date.getFullYear() === year &&
      date.getMonth() === month - 1 &&
      date.getDate() === day
    ){
      return date;
    }
  }

  return getSafeDate(normalized);
}

function getInvoiceDate(invoice){
  return parseInvoiceDate(invoice.date) ||
    getSafeDate(invoice.createdAt) ||
    getSafeDate(invoice.sortTimestamp) ||
    getSafeDate(invoice.updatedAt);
}

function isSameDay(firstDate, secondDate){
  if(!firstDate || !secondDate) return false;
  return (
    firstDate.getFullYear() === secondDate.getFullYear() &&
    firstDate.getMonth() === secondDate.getMonth() &&
    firstDate.getDate() === secondDate.getDate()
  );
}

function safeMoney(value){
  const amount = Number(value);
  return Number.isFinite(amount) && amount >= 0 ? amount : 0;
}

function formatMoney(value){
  return `${moneyFormatter.format(safeMoney(value))} ج.م`;
}

function formatDate(value){
  const date = value instanceof Date ? value : getSafeDate(value);
  return date ? dateFormatter.format(date) : "غير متوفر";
}

function normalizeStatus(value){
  return ["pending", "shipped", "completed"].includes(value)
    ? value
    : "pending";
}

function getStatusLabel(status){
  return {
    pending:"قيد الانتظار",
    shipped:"تم الشحن",
    completed:"مكتمل"
  }[normalizeStatus(status)];
}

function getStockQuantity(product){
  if(product?.stockQty === null || product?.stockQty === undefined || product?.stockQty === ""){
    return null;
  }

  const quantity = Number(product.stockQty);
  return Number.isFinite(quantity) ? Math.max(0, quantity) : null;
}

function renderState(container, message, type = "empty"){
  container.replaceChildren();
  const state = document.createElement("div");
  state.className = type === "error" ? "mobile-error-state" : "mobile-empty-state";
  state.setAttribute("role", type === "error" ? "alert" : "status");
  state.textContent = message;
  container.appendChild(state);
}

function updateSyncState(snapshot){
  if(snapshot.metadata.hasPendingWrites){
    syncTime.textContent = "جاري إرسال التعديل…";
    return;
  }

  if(snapshot.metadata.fromCache){
    syncTime.textContent = navigator.onLine
      ? "جاري تحديث البيانات…"
      : "آخر بيانات في الجلسة الحالية";
    return;
  }

  syncTime.textContent = `آخر مزامنة ${new Date().toLocaleTimeString("ar-EG", {
    hour:"2-digit",
    minute:"2-digit"
  })}`;
}

function updateReadyState(){
  const requiredStates = currentRole === "owner"
    ? [loadStates.orders, loadStates.invoices, loadStates.products]
    : [loadStates.orders, loadStates.invoices];
  const allSettled = requiredStates.every((state) => state !== "loading");

  dashboardContent.setAttribute("aria-busy", String(!allSettled));
  if(allSettled){
    appStatus.textContent = navigator.onLine
      ? "كل التعديلات هنا مرتبطة مباشرة بلوحة الموقع."
      : "الاتصال مقطوع؛ العرض الحالي قديم والتعديل متوقف.";
  }
}

function updateOrderStatistics(){
  const now = new Date();
  const pendingCount = allOrders.filter((order) => (
    normalizeStatus(order.status) === "pending"
  )).length;
  const todaySales = allOrders.reduce((sum, order) => {
    const orderDate = getSafeDate(order.createdAt);
    return normalizeStatus(order.status) === "completed" && isSameDay(orderDate, now)
      ? sum + safeMoney(order.total)
      : sum;
  }, 0);

  document.getElementById("statPendingOrders").textContent = countFormatter.format(pendingCount);
  document.getElementById("statTodaySales").textContent = formatMoney(todaySales);
}

function updateInvoiceStatistics(){
  const now = new Date();
  const todayCount = allInvoices.filter((invoice) => (
    isSameDay(getInvoiceDate(invoice), now)
  )).length;

  document.getElementById("statTodayInvoices").textContent = countFormatter.format(todayCount);
}

function updateStockStatistics(){
  const displayedCount = lowStockProducts.length;
  document.getElementById("statLowStock").textContent =
    displayedCount >= 50 ? "٥٠+" : countFormatter.format(displayedCount);
}

function createOrderCard(order){
  const status = normalizeStatus(order.status);
  const orderId = String(order.docId || "");
  const customerName = String(order.customerName || "عميل بدون اسم");
  const phone = String(order.phone || "لا يوجد رقم هاتف");
  const createdAt = getSafeDate(order.createdAt);

  const card = document.createElement("article");
  card.className = "mobile-order-card";

  const top = document.createElement("div");
  top.className = "mobile-order-top";
  const idLabel = document.createElement("bdi");
  idLabel.dir = "ltr";
  idLabel.className = "mobile-order-id";
  idLabel.textContent = `#${orderId.slice(0, 8).toUpperCase() || "—"}`;
  const statusBadge = document.createElement("span");
  statusBadge.className = `mobile-status-badge ${status}`;
  statusBadge.textContent = getStatusLabel(status);
  top.append(idLabel, statusBadge);

  const main = document.createElement("div");
  main.className = "mobile-order-main";
  const customer = document.createElement("strong");
  customer.textContent = customerName;
  const total = document.createElement("span");
  total.className = "mobile-order-total";
  total.textContent = formatMoney(order.total);
  main.append(customer, total);

  const meta = document.createElement("p");
  meta.className = "mobile-order-meta";
  const phoneBdi = document.createElement("bdi");
  phoneBdi.dir = "ltr";
  phoneBdi.textContent = phone;
  meta.append("الهاتف: ", phoneBdi, document.createElement("br"), `التاريخ: ${formatDate(createdAt)}`);

  const footer = document.createElement("div");
  footer.className = "mobile-order-footer";
  const statusSelect = document.createElement("select");
  statusSelect.dataset.orderId = orderId;
  statusSelect.dataset.previousStatus = status;
  statusSelect.disabled = !navigator.onLine;
  statusSelect.setAttribute("aria-label", `تغيير حالة طلب ${customerName}`);

  [
    ["pending", "قيد الانتظار"],
    ["shipped", "تم الشحن"],
    ["completed", "مكتمل"]
  ].forEach(([value, label]) => {
    const option = document.createElement("option");
    option.value = value;
    option.textContent = label;
    option.selected = value === status;
    statusSelect.appendChild(option);
  });

  const openLink = document.createElement("a");
  openLink.href = "admin.html";
  openLink.textContent = "فتح التفاصيل ←";
  footer.append(statusSelect, openLink);

  card.append(top, main, meta, footer);
  return card;
}

function renderRecentOrders(){
  if(!allOrders.length){
    renderState(recentOrdersContainer, "مفيش طلبات حاليًا.");
    return;
  }

  recentOrdersContainer.replaceChildren();
  const fragment = document.createDocumentFragment();
  allOrders.slice(0, 5).forEach((order) => fragment.appendChild(createOrderCard(order)));
  recentOrdersContainer.appendChild(fragment);
}

function createInvoiceRow(invoice){
  const invoiceId = String(invoice.docId || "");
  const number = String(invoice.number || "بدون رقم");
  const customer = String(invoice.customer || "عميل غير محدد");
  const date = getInvoiceDate(invoice);

  const link = document.createElement("a");
  link.className = "mobile-invoice-row";
  link.href = `new_invoice.html?id=${encodeURIComponent(invoiceId)}`;
  link.setAttribute("aria-label", `فتح الفاتورة ${number} للعميل ${customer}`);

  const copy = document.createElement("span");
  const numberElement = document.createElement("strong");
  const numberBdi = document.createElement("bdi");
  numberBdi.dir = "ltr";
  numberBdi.textContent = number;
  numberElement.appendChild(numberBdi);
  const details = document.createElement("small");
  details.textContent = `${customer} • ${formatDate(date)}`;
  copy.append(numberElement, details);

  const amount = document.createElement("span");
  amount.className = "mobile-invoice-amount";
  amount.textContent = formatMoney(invoice.total);
  link.append(copy, amount);
  return link;
}

function renderRecentInvoices(){
  if(!allInvoices.length){
    renderState(recentInvoicesContainer, "مفيش فواتير محفوظة لسه.");
    return;
  }

  recentInvoicesContainer.replaceChildren();
  const fragment = document.createDocumentFragment();
  allInvoices.slice(0, 4).forEach((invoice) => fragment.appendChild(createInvoiceRow(invoice)));
  recentInvoicesContainer.appendChild(fragment);
}

function createStockRow(product){
  const quantity = getStockQuantity(product);
  const link = document.createElement("a");
  link.className = "mobile-stock-row";
  link.href = "inventory-dashboard.html";

  const copy = document.createElement("span");
  const name = document.createElement("strong");
  name.textContent = String(product.name || "منتج بدون اسم");
  const category = document.createElement("small");
  category.textContent = String(product.category || "بدون تصنيف");
  copy.append(name, category);

  const quantityBadge = document.createElement("span");
  quantityBadge.className = "mobile-stock-qty";
  quantityBadge.textContent = quantity === null
    ? "غير محدد"
    : `${countFormatter.format(quantity)} قطع`;

  link.append(copy, quantityBadge);
  return link;
}

function renderLowStockProducts(){
  if(!lowStockProducts.length){
    renderState(lowStockContainer, "المخزون كويس ومفيش تنبيهات رقمية.");
    return;
  }

  lowStockContainer.replaceChildren();
  const fragment = document.createDocumentFragment();
  lowStockProducts.slice(0, 5).forEach((product) => fragment.appendChild(createStockRow(product)));
  lowStockContainer.appendChild(fragment);
}

function subscribeToOrders(){
  loadStates.orders = "loading";
  const unsubscribe = onSnapshot(
    collection(db, "orders"),
    { includeMetadataChanges:true },
    (snapshot) => {
      allOrders = snapshot.docs
        .map((snapshotDoc) => ({ ...snapshotDoc.data(), docId:snapshotDoc.id }))
        .sort((first, second) => (
          (getSafeDate(second.createdAt)?.getTime() || 0) -
          (getSafeDate(first.createdAt)?.getTime() || 0)
        ));

      loadStates.orders = "ready";
      updateOrderStatistics();
      renderRecentOrders();
      updateSyncState(snapshot);
      updateReadyState();
    },
    (error) => {
      console.error("Mobile orders listener failed:", error);
      loadStates.orders = "error";
      allOrders = [];
      document.getElementById("statPendingOrders").textContent = "—";
      document.getElementById("statTodaySales").textContent = "—";
      renderState(recentOrdersContainer, "تعذر تحميل الطلبات. تأكد من الصلاحيات والاتصال.", "error");
      updateReadyState();
    }
  );
  unsubscribeListeners.push(unsubscribe);
}

function subscribeToInvoices(){
  loadStates.invoices = "loading";
  const unsubscribe = onSnapshot(
    collection(db, "invoices"),
    { includeMetadataChanges:true },
    (snapshot) => {
      allInvoices = snapshot.docs
        .map((snapshotDoc) => ({ ...snapshotDoc.data(), docId:snapshotDoc.id }))
        .sort((first, second) => (
          (getInvoiceDate(second)?.getTime() || 0) -
          (getInvoiceDate(first)?.getTime() || 0)
        ));

      loadStates.invoices = "ready";
      updateInvoiceStatistics();
      renderRecentInvoices();
      updateSyncState(snapshot);
      updateReadyState();
    },
    (error) => {
      console.error("Mobile invoices listener failed:", error);
      loadStates.invoices = "error";
      allInvoices = [];
      document.getElementById("statTodayInvoices").textContent = "—";
      renderState(recentInvoicesContainer, "تعذر تحميل الفواتير. تأكد من الصلاحيات والاتصال.", "error");
      updateReadyState();
    }
  );
  unsubscribeListeners.push(unsubscribe);
}

function subscribeToLowStock(){
  loadStates.products = "loading";
  const lowStockQuery = query(
    collection(db, "products"),
    where("stockQty", "<=", 5),
    limit(50)
  );

  const unsubscribe = onSnapshot(
    lowStockQuery,
    { includeMetadataChanges:true },
    (snapshot) => {
      lowStockProducts = snapshot.docs
        .map((snapshotDoc) => ({ ...snapshotDoc.data(), docId:snapshotDoc.id }))
        .filter((product) => product.deleted !== true)
        .sort((first, second) => (
          (getStockQuantity(first) ?? Number.MAX_SAFE_INTEGER) -
          (getStockQuantity(second) ?? Number.MAX_SAFE_INTEGER)
        ));

      loadStates.products = "ready";
      updateStockStatistics();
      renderLowStockProducts();
      updateSyncState(snapshot);
      updateReadyState();
    },
    (error) => {
      console.error("Mobile stock listener failed:", error);
      loadStates.products = "error";
      lowStockProducts = [];
      document.getElementById("statLowStock").textContent = "—";
      renderState(lowStockContainer, "تعذر تحميل تنبيهات المخزون.", "error");
      updateReadyState();
    }
  );
  unsubscribeListeners.push(unsubscribe);
}

async function changeOrderStatus(select){
  const orderId = select.dataset.orderId;
  const previousStatus = normalizeStatus(select.dataset.previousStatus);
  const newStatus = normalizeStatus(select.value);

  if(!navigator.onLine){
    select.value = previousStatus;
    showToast("رجّع الإنترنت الأول علشان نضمن وصول التعديل.", "warning");
    return;
  }

  if(!orderId || newStatus === previousStatus) return;

  select.disabled = true;
  try{
    await updateDoc(doc(db, "orders", orderId), { status:newStatus });
    select.dataset.previousStatus = newStatus;
    showToast("تم تحديث حالة الطلب وظهرت على لوحة الموقع.");
  }catch(error){
    console.error("Mobile order status update failed:", error);
    select.value = previousStatus;
    showToast(
      String(error?.code || "") === "permission-denied"
        ? "الحساب ده مش مسموح له بتعديل الطلب."
        : "تعذر تحديث الطلب. جرّب مرة تانية.",
      "error",
      5000
    );
  }finally{
    if(select.isConnected) select.disabled = !navigator.onLine;
  }
}

recentOrdersContainer.addEventListener("change", (event) => {
  const select = event.target.closest("select[data-order-id]");
  if(select) changeOrderStatus(select);
});

window.addEventListener("kimo-connectivity-change", (event) => {
  const online = Boolean(event.detail?.online);
  recentOrdersContainer.querySelectorAll("select[data-order-id]").forEach((select) => {
    select.disabled = !online;
  });

  if(!online){
    appStatus.textContent = "الاتصال مقطوع؛ العرض الحالي قديم والتعديل متوقف.";
  }else if(currentRole){
    appStatus.textContent = "رجع الاتصال وجاري مزامنة أحدث البيانات…";
  }
});

logoutButton.addEventListener("click", async () => {
  logoutButton.disabled = true;
  try{
    await signOut(auth);
    redirectToLogin();
  }catch(error){
    console.error("Mobile logout failed:", error);
    logoutButton.disabled = false;
    showToast("تعذر تسجيل الخروج. جرّب مرة تانية.", "error");
  }
});

onAuthStateChanged(auth, async (user) => {
  if(!user){
    redirectToLogin("سجّل الدخول علشان تفتح تطبيق الإدارة.");
    return;
  }

  try{
    const userSnapshot = await getDoc(doc(db, "users", user.uid));
    const role = userSnapshot.exists()
      ? String(userSnapshot.data().role || "").trim().toLowerCase()
      : "";

    if(!allowedRoles.has(role)){
      await signOut(auth).catch(()=>{});
      redirectToLogin("الحساب ده غير مصرح له بدخول لوحة الإدارة.");
      return;
    }

    currentRole = role;
    sessionStorage.setItem("adminRole", role);
    window.setMobileAdminRole?.(role);

    document.getElementById("mobileRoleLabel").textContent =
      role === "owner" ? "صلاحية المالك" : "إدارة الطلبات";
    document.getElementById("mobileWelcomeTitle").textContent =
      `أهلاً ${user.displayName || user.email?.split("@")[0] || "بك"}`;

    document.querySelectorAll("[data-mobile-owner-only]").forEach((element) => {
      element.hidden = role !== "owner";
    });

    loadingCard.hidden = true;
    dashboardContent.hidden = false;
    dashboardContent.setAttribute("aria-busy", "true");
    appStatus.textContent = "تم التحقق من الحساب وجاري تحميل البيانات…";

    subscribeToOrders();
    subscribeToInvoices();
    if(role === "owner"){
      subscribeToLowStock();
    }else{
      loadStates.products = "idle";
    }
  }catch(error){
    console.error("Mobile admin authorization failed:", error);
    loadingCard.hidden = false;
    loadingCard.querySelector("strong").textContent = "تعذر التحقق من الحساب";
    loadingCard.querySelector("p").textContent =
      "تأكد من الاتصال ثم أعد المحاولة، أو سجّل الدخول مرة ثانية.";
    document.getElementById("retryMobileData").hidden = false;
  }
});

window.addEventListener("beforeunload", () => {
  unsubscribeListeners.forEach((unsubscribe) => unsubscribe());
  unsubscribeListeners = [];
});
