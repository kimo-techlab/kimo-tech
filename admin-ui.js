(() => {
  const page = (window.location.pathname.split("/").pop() || "admin.html").toLowerCase();
  document.body.classList.add("admin-page");

  if(page === "admin-login.html"){
    document.body.classList.add("admin-login-page");
    return;
  }

  const pages = [
    { href:"mobile-admin.html", label:"تطبيق الموبايل", icon:"📱" },
    { href:"admin.html", label:"الطلبات", icon:"📋" },
    { href:"products-admin.html", label:"المنتجات", icon:"📦", ownerOnly:true },
    { href:"projects-admin.html", label:"المشروعات", icon:"🧠", ownerOnly:true },
    { href:"inventory-dashboard.html", label:"المخزون", icon:"📊", ownerOnly:true },
    { href:"dashboard.html", label:"الفواتير", icon:"🧾" },
    { href:"new_invoice.html?mode=new", match:"new_invoice.html", label:"فاتورة جديدة", icon:"➕" }
  ];

  const nav = document.createElement("nav");
  nav.className = "admin-global-nav";
  nav.setAttribute("aria-label", "التنقل بين صفحات الإدارة");

  const navInner = document.createElement("div");
  navInner.className = "admin-nav-inner";

  const brand = document.createElement("a");
  brand.className = "admin-brand";
  brand.href = "admin.html";
  brand.setAttribute("aria-label", "لوحة إدارة KIMO TECH");
  brand.innerHTML = '<span class="admin-brand-mark" aria-hidden="true">K</span><strong>KIMO <span>ADMIN</span></strong>';

  const links = document.createElement("div");
  links.className = "admin-nav-links";

  pages.forEach((item) => {
    const link = document.createElement("a");
    link.className = "admin-nav-link";
    link.href = item.href;
    link.innerHTML = `<span aria-hidden="true">${item.icon}</span>${item.label}`;
    if(item.ownerOnly) link.dataset.ownerOnly = "true";

    if(page === (item.match || item.href)){
      link.classList.add("active");
      link.setAttribute("aria-current", "page");
    }

    links.appendChild(link);
  });

  const siteLink = document.createElement("a");
  siteLink.className = "admin-nav-link admin-site-link";
  siteLink.href = "index.html";
  siteLink.innerHTML = '<span aria-hidden="true">↗</span>عرض الموقع';
  links.appendChild(siteLink);

  const logoutLink = document.createElement("a");
  logoutLink.className = "admin-nav-link admin-logout-link";
  logoutLink.href = "admin-login.html?logout=1";
  logoutLink.innerHTML = '<span aria-hidden="true">⇥</span>تسجيل الخروج';
  links.appendChild(logoutLink);

  const applyNavigationRole = (role) => {
    const normalizedRole = String(role || "").trim().toLowerCase();
    links.querySelectorAll("[data-owner-only]").forEach((link) => {
      link.hidden = normalizedRole === "admin";
    });
    document.querySelectorAll("[data-admin-owner-only]").forEach((element) => {
      element.hidden = normalizedRole === "admin";
    });
  };

  window.setAdminNavigationRole = (role) => {
    const normalizedRole = String(role || "").trim().toLowerCase();
    if(normalizedRole) sessionStorage.setItem("adminRole", normalizedRole);
    applyNavigationRole(normalizedRole);
  };

  applyNavigationRole(sessionStorage.getItem("adminRole"));

  navInner.append(brand, links);
  nav.appendChild(navInner);
  const skipLink = document.querySelector(".admin-skip-link");
  if(skipLink){
    skipLink.after(nav);
  }else{
    document.body.prepend(nav);
  }

  if(page === "products-admin.html"){
    const toolsPanel = document.querySelector(".assistant-panel");

    if(toolsPanel && !toolsPanel.closest(".admin-tools-disclosure")){
      const disclosure = document.createElement("details");
      disclosure.className = "admin-tools-disclosure";

      const summary = document.createElement("summary");
      summary.innerHTML = '<span>أدوات الصور والاستيراد المتقدمة</span><small>للمالك فقط</small>';

      toolsPanel.before(disclosure);
      toolsPanel.classList.add("admin-tools-panel");
      disclosure.append(summary, toolsPanel);
      disclosure.addEventListener("toggle", () => {
        if(disclosure.open) window.loadMakersCatalog?.();
      });
    }
  }

  const tableConfigs = {
    "products-admin.html": {
      selector:"#productsContainer",
      labels:["الترتيب","الصورة","الاسم","السعر","التصنيف","الحالة","الإجراءات"]
    },
    "projects-admin.html": {
      selector:"#projectsContainer",
      labels:["الترتيب","الصورة","الاسم","السعر","التصنيف","الحالة","الإجراءات"]
    },
    "dashboard.html": {
      selector:"#invoiceContainer",
      labels:["رقم الفاتورة","بيانات العميل","الإجمالي","التاريخ","الإجراءات"]
    }
  };

  const config = tableConfigs[page];

  if(config){
    const container = document.querySelector(config.selector);
    const table = container?.closest(".table");

    if(table){
      table.classList.add("admin-mobile-cards");
    }

    const enhanceRows = () => {
      container?.querySelectorAll(".table-row").forEach((row) => {
        [...row.children].forEach((cell, index) => {
          cell.dataset.label = config.labels[index] || "";
        });
      });
    };

    enhanceRows();

    if(container){
      new MutationObserver(enhanceRows).observe(container, { childList:true, subtree:true });
    }
  }
})();
