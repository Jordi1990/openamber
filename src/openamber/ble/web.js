(() => {
  function init() {
    if (document.getElementById("openamber-ble")) return;
    if (!document.querySelector("meta[name=viewport]")) {
      const viewport = document.createElement("meta");
      viewport.name = "viewport";
      viewport.content = "width=device-width, initial-scale=1";
      document.head.append(viewport);
    }
    const host = document.createElement("section");
    host.id = "openamber-ble";
    const root = host.attachShadow({ mode: "open" });
    root.innerHTML = `
      <link rel="stylesheet" href="/0.css">
      <div class="tab-header">Ruimtetemperatuur</div>
      <div class="tab-container">
        <label
          >Temperatuurbron
          <select id="source" aria-label="Ruimtetemperatuurbron" disabled>
            <option>Tr</option>
            <option>BLE</option>
          </select>
        </label>
        <div id="ble" hidden>
          <p id="selected"></p>
          <label
            >Meting geldig
            <select id="timeout" aria-label="BLE meettimeout" disabled></select>
          </label>
          <button id="refresh" type="button">Sensor zoeken</button>
          <div id="status" role="status"></div>
          <div id="list"></div>
        </div>
        <div id="error" role="alert" hidden></div>
      </div>
    `;
    document.body.append(host);
    // Reuse the stock v3 column and styles, including its light/dark toggle.
    // Keep the panel usable if the externally hosted stock UI is unavailable.
    customElements.whenDefined("esp-app").then(async () => {
      const app = document.querySelector("esp-app");
      await app?.updateComplete;
      const column = app?.shadowRoot?.querySelector("#col_entities");
      const table = column?.querySelector("esp-entity-table");
      await table?.updateComplete;
      if (table?.shadowRoot)
        root.adoptedStyleSheets = [...table.shadowRoot.adoptedStyleSheets];
      if (column) column.prepend(host);
    });
    const source = root.getElementById("source"),
      ble = root.getElementById("ble");
    const timeout = root.getElementById("timeout");
    for (let minutes = 1; minutes <= 15; minutes++)
      timeout.add(new Option(`${minutes} min`, minutes));
    const selected = root.getElementById("selected"),
      status = root.getElementById("status");
    const list = root.getElementById("list"),
      refresh = root.getElementById("refresh");
    const error = root.getElementById("error");
    const rows = new Map();
    let busy = false,
      polling = false,
      timer,
      failed = false;
    let commandError = "";
    function describe(reading) {
      return `${reading.temperature.toFixed(1).replace(".", ",")} °C  |  ${reading.humidity}% luchtvochtigheid  |  ${reading.age}s geleden`;
    }
    function render(data) {
      source.value = data.source;
      timeout.value = data.timeout_minutes || 5;
      ble.hidden = data.source === "Tr";
      selected.textContent = data.mac
        ? `SwitchBot  ${data.mac}\n${data.reading ? describe(data.reading) : "Geen actuele meting"}`
        : "Geen sensor gekozen";
      refresh.textContent = data.mac ? "Andere sensor zoeken" : "Sensor zoeken";
      refresh.disabled = busy || data.searching;
      status.textContent = data.searching
        ? `Zoeken: ${data.remaining}s resterend · ${data.devices.length} sensoren gevonden`
        : `${data.devices.length} sensoren gevonden`;
      const found = new Set();
      for (const device of data.devices.slice(0, 8)) {
        found.add(device.mac);
        let row = rows.get(device.mac);
        if (!row) {
          row = document.createElement("button");
          row.type = "button";
          row.className = "device";
          row.innerHTML =
            "<strong>SwitchBot</strong><small></small><span></span>";
          row.querySelector("small").textContent = device.mac;
          row.dataset.mac = device.mac;
          rows.set(device.mac, row);
          list.append(row);
        }
        row.querySelector("span").textContent = describe(device);
        row.setAttribute("aria-pressed", String(device.mac === data.mac));
        row.disabled = busy;
      }
      for (const [mac, row] of rows)
        if (!found.has(mac)) {
          row.remove();
          rows.delete(mac);
        }
    }
    function disableControls(disabled) {
      root.querySelectorAll("select, button").forEach((control) => {
        control.disabled = disabled;
      });
    }
    async function request(url, method = "GET") {
      const response = await fetch(url, {
        method,
        cache: "no-store",
        signal: AbortSignal.timeout(5000),
      });
      if (!response.ok) throw new Error("HTTP " + response.status);
      return response;
    }
    async function update() {
      clearTimeout(timer);
      if (polling || busy) return;
      polling = true;
      if (!document.hidden) {
        try {
          const response = await request("/text_sensor/BLE%20sensorlijst");
          const data = JSON.parse((await response.json()).state);
          // A setting may have changed while this request was in flight.
          if (!busy) {
            disableControls(false);
            render(data);
          }
          error.textContent = commandError;
          error.hidden = !commandError;
          failed = false;
        } catch (e) {
          failed = true;
          disableControls(true);
          selected.textContent = "Geen actuele meting (verbinding verbroken)";
          error.textContent =
            "Geen actuele verbinding met OpenAmber. Opnieuw verbinden…";
          error.hidden = false;
        }
      }
      polling = false;
      timer = setTimeout(update, failed ? 5000 : ble.hidden ? 10000 : 2000);
    }
    async function command(url) {
      if (busy) return;
      busy = true;
      commandError = "";
      disableControls(true);
      clearTimeout(timer);
      try {
        await request(url, "POST");
        // The firmware publishes a new sensor-list snapshot every two seconds.
        await new Promise((resolve) => setTimeout(resolve, 2300));
      } catch (e) {
        commandError =
          "Instelling kon niet worden opgeslagen. Controleer de verbinding.";
      } finally {
        busy = false;
        update();
      }
    }
    list.addEventListener("click", (event) => {
      const row = event.target.closest("button[data-mac]");
      if (row)
        command(
          "/text/SwitchBot%20MAC%20Adres/set?value=" +
            encodeURIComponent(row.dataset.mac),
        );
    });
    source.addEventListener("change", () =>
      command(
        "/select/Ruimtetemperatuurbron/set?option=" +
          encodeURIComponent(source.value),
      ),
    );
    timeout.addEventListener("change", () =>
      command(
        "/number/BLE%20meettimeout/set?value=" +
          encodeURIComponent(timeout.value),
      ),
    );
    refresh.addEventListener("click", () =>
      command("/button/BLE%20sensoren%20zoeken/press"),
    );
    document.addEventListener("visibilitychange", () => {
      if (!document.hidden && !busy) update();
    });
    update();
  }
  if (document.readyState === "loading")
    document.addEventListener("DOMContentLoaded", init, { once: true });
  else init();
})();
