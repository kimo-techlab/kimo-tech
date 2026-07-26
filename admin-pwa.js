(() => {
  const pageFromPath = (window.location.pathname.split("/").pop() || "").toLowerCase();
  const page = document.body?.dataset.adminPage === "mobile-home"
    ? "mobile-admin.html"
    : pageFromPath || "mobile-admin.html";
  const isLoginPage = page === "admin-login.html";
  const installButton = document.getElementById("installAppButton");
  const installGuide = document.getElementById("installGuide");
  const installGuideText = document.getElementById("installGuideText");
  const closeInstallGuideButton = document.getElementById("closeInstallGuide");
  const allowedServiceWorkerHosts = new Set([
    "admin.kimo-techlab.com",
    "localhost",
    "127.0.0.1"
  ]);

  let deferredInstallPrompt = null;
  let dataModulePromise = null;
  let mobileMoreTrigger = null;

  const isStandalone = () => (
    window.matchMedia("(display-mode: standalone)").matches ||
    window.navigator.standalone === true
  );

  function registerServiceWorker(){
    if(
      !("serviceWorker" in navigator) ||
      !window.isSecureContext ||
      !allowedServiceWorkerHosts.has(window.location.hostname)
    ){
      return;
    }

    window.addEventListener("load", () => {
      navigator.serviceWorker.register("/admin-sw.js", { scope:"/" })
        .catch((error) => console.error("Admin PWA registration failed:", error));
    }, { once:true });
  }

  function getInstallHelpMessage(){
    const isIos = /iphone|ipad|ipod/i.test(navigator.userAgent);
    if(isIos){
      return "من Safari اضغط زر المشاركة، ثم اختر «إضافة إلى الشاشة الرئيسية» وبعدها «إضافة».";
    }

    if(!window.isSecureContext){
      return "تثبيت التطبيق يعمل بعد رفع لوحة الإدارة على رابط HTTPS الآمن.";
    }

    return "افتح قائمة المتصفح واختر «تثبيت التطبيق» أو «إضافة إلى الشاشة الرئيسية».";
  }

  function openInstallGuide(){
    if(!installGuide) return;
    if(installGuideText) installGuideText.textContent = getInstallHelpMessage();

    if(typeof installGuide.showModal === "function"){
      installGuide.showModal();
      closeInstallGuideButton?.focus();
    }else{
      installGuide.setAttribute("open", "");
    }
  }

  function setupInstallPrompt(){
    if(isStandalone()){
      if(installButton) installButton.hidden = true;
      return;
    }

    const isIos = /iphone|ipad|ipod/i.test(navigator.userAgent);
    if(installButton && isIos) installButton.hidden = false;

    window.addEventListener("beforeinstallprompt", (event) => {
      event.preventDefault();
      deferredInstallPrompt = event;
      if(installButton) installButton.hidden = false;
    });

    window.addEventListener("appinstalled", () => {
      deferredInstallPrompt = null;
      if(installButton) installButton.hidden = true;
      window.showMobileAdminToast?.("تم تثبيت تطبيق KIMO Admin بنجاح.");
    });

    installButton?.addEventListener("click", async () => {
      if(!deferredInstallPrompt){
        openInstallGuide();
        return;
      }

      installButton.disabled = true;
      try{
        await deferredInstallPrompt.prompt();
        await deferredInstallPrompt.userChoice;
        deferredInstallPrompt = null;
        installButton.hidden = true;
      }finally{
        installButton.disabled = false;
      }
    });

    closeInstallGuideButton?.addEventListener("click", () => {
      if(typeof installGuide?.close === "function") installGuide.close();
      else installGuide?.removeAttribute("open");
    });

    installGuide?.addEventListener("click", (event) => {
      if(event.target !== installGuide) return;
      if(typeof installGuide.close === "function") installGuide.close();
      else installGuide.removeAttribute("open");
    });
  }

  function setConnectivityState(){
    const online = navigator.onLine;
    document.documentElement.dataset.online = String(online);
    document.body.classList.toggle("pwa-offline", !online);

    const existingBanner = document.getElementById("offlineBanner");
    if(existingBanner){
      existingBanner.hidden = online;
    }else if(!online){
      const banner = document.createElement("div");
      banner.id = "offlineBanner";
      banner.className = "admin-pwa-connectivity";
      banner.setAttribute("role", "status");
      banner.textContent = "مفيش اتصال — التعديلات متوقفة لحد رجوع الإنترنت";
      document.body.appendChild(banner);
    }else{
      document.getElementById("offlineBanner")?.remove();
    }

    const connectionLabel = document.getElementById("connectionLabel");
    if(connectionLabel){
      connectionLabel.textContent = online ? "متصل لحظيًا" : "بدون إنترنت";
      connectionLabel.closest(".mobile-connection-pill")
        ?.classList.toggle("offline", !online);
    }

    window.dispatchEvent(new CustomEvent("kimo-connectivity-change", {
      detail:{ online }
    }));

    if(online) loadMobileDataModule();
  }

  async function loadMobileDataModule(){
    if(page !== "mobile-admin.html" || !navigator.onLine) return;
    if(dataModulePromise) return dataModulePromise;

    const appStatus = document.getElementById("appDataStatus");
    if(appStatus) appStatus.textContent = "جاري الاتصال ببيانات الإدارة…";

    dataModulePromise = import("./mobile-admin-data.js")
      .catch((error) => {
        console.error("Mobile admin data module failed:", error);
        dataModulePromise = null;
        document.getElementById("mobileAppLoading")?.removeAttribute("hidden");
        const loadingText = document.querySelector("#mobileAppLoading p");
        if(loadingText){
          loadingText.textContent = "تعذر تشغيل بيانات التطبيق. تأكد من الإنترنت ثم اضغط إعادة المحاولة.";
        }
        const retryButton = document.getElementById("retryMobileData");
        if(retryButton) retryButton.hidden = false;
      });

    return dataModulePromise;
  }

  function setMobileRole(role){
    const normalizedRole = String(role || "").trim().toLowerCase();
    document.querySelectorAll("[data-mobile-owner-only]").forEach((element) => {
      element.hidden = normalizedRole !== "owner";
    });
  }

  function setupRoleBridge(){
    window.setMobileAdminRole = setMobileRole;
    setMobileRole(sessionStorage.getItem("adminRole"));

    const originalRoleSetter = window.setAdminNavigationRole;
    if(typeof originalRoleSetter === "function"){
      window.setAdminNavigationRole = (role) => {
        originalRoleSetter(role);
        setMobileRole(role);
      };
    }
  }

  function createMobileNavigation(){
    if(isLoginPage || document.querySelector(".admin-mobile-tabbar")) return;

    const tabbar = document.createElement("nav");
    tabbar.className = "admin-mobile-tabbar";
    tabbar.setAttribute("aria-label", "التنقل في تطبيق الإدارة");

    const destinations = [
      { href:"mobile-admin.html", match:"mobile-admin.html", icon:"⌂", label:"الرئيسية" },
      { href:"admin.html", match:"admin.html", icon:"▤", label:"الطلبات" },
      { href:"new_invoice.html?mode=new", match:"new_invoice.html", icon:"＋", label:"فاتورة", primary:true },
      { href:"dashboard.html", match:"dashboard.html", icon:"▧", label:"الفواتير" }
    ];

    destinations.forEach((destination) => {
      const link = document.createElement("a");
      link.href = destination.href;
      link.className = destination.primary ? "admin-mobile-new-invoice" : "";
      link.innerHTML = `
        <span class="admin-mobile-tab-icon" aria-hidden="true">${destination.icon}</span>
        <span class="admin-mobile-tab-label">${destination.label}</span>
      `;

      if(page === destination.match){
        link.setAttribute("aria-current", "page");
      }

      tabbar.appendChild(link);
    });

    const moreButton = document.createElement("button");
    moreButton.type = "button";
    moreButton.setAttribute("aria-haspopup", "dialog");
    moreButton.setAttribute("aria-expanded", "false");
    moreButton.innerHTML = `
      <span class="admin-mobile-tab-icon" aria-hidden="true">•••</span>
      <span class="admin-mobile-tab-label">المزيد</span>
    `;
    tabbar.appendChild(moreButton);

    const layer = document.createElement("div");
    layer.className = "admin-mobile-more-layer";
    layer.hidden = true;
    layer.innerHTML = `
      <section class="admin-mobile-more-sheet" role="dialog" aria-modal="true" aria-labelledby="mobileMoreTitle">
        <header class="admin-mobile-more-head">
          <h2 id="mobileMoreTitle">باقي أدوات الإدارة</h2>
          <button type="button" class="admin-mobile-more-close" aria-label="إغلاق القائمة">×</button>
        </header>
        <nav class="admin-mobile-more-links" aria-label="باقي صفحات الإدارة">
          <a href="products-admin.html" data-mobile-owner-only hidden><span aria-hidden="true">📦</span>المنتجات والأسعار</a>
          <a href="inventory-dashboard.html" data-mobile-owner-only hidden><span aria-hidden="true">📊</span>المخزون</a>
          <a href="projects-admin.html" data-mobile-owner-only hidden><span aria-hidden="true">🧠</span>المشروعات</a>
          <a href="https://community.kimo-techlab.com/admin.html" target="_blank" rel="noopener"><span aria-hidden="true">👥</span>إدارة المجتمع</a>
          <a href="index.html"><span aria-hidden="true">↗</span>عرض الموقع</a>
          <a href="admin-login.html?logout=1"><span aria-hidden="true">⇥</span>تسجيل الخروج</a>
        </nav>
      </section>
    `;

    const closeButton = layer.querySelector(".admin-mobile-more-close");
    const closeLayer = () => {
      layer.hidden = true;
      moreButton.setAttribute("aria-expanded", "false");
      document.body.style.overflow = "";
      mobileMoreTrigger?.focus();
      mobileMoreTrigger = null;
    };

    moreButton.addEventListener("click", () => {
      mobileMoreTrigger = moreButton;
      layer.hidden = false;
      moreButton.setAttribute("aria-expanded", "true");
      document.body.style.overflow = "hidden";
      closeButton.focus();
    });
    closeButton.addEventListener("click", closeLayer);
    layer.addEventListener("click", (event) => {
      if(event.target === layer) closeLayer();
    });
    layer.querySelectorAll("a").forEach((link) => {
      link.addEventListener("click", () => {
        document.body.style.overflow = "";
      });
    });
    document.addEventListener("keydown", (event) => {
      if(event.key === "Escape" && !layer.hidden) closeLayer();
    });

    document.body.append(layer, tabbar);
    document.body.classList.add("pwa-mobile-nav-ready");
    setMobileRole(sessionStorage.getItem("adminRole"));
  }

  registerServiceWorker();
  setupInstallPrompt();
  setupRoleBridge();
  createMobileNavigation();
  setConnectivityState();

  window.addEventListener("online", setConnectivityState);
  window.addEventListener("offline", setConnectivityState);
  document.getElementById("retryMobileData")?.addEventListener("click", () => {
    window.location.reload();
  });
})();
