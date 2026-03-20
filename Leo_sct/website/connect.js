const SESSION_ENDPOINT = "/api/session";
const CONNECT_INFO_ENDPOINT = "/api/connect_info";
const QR_ENDPOINT = "/api/qr";

const phoneConnectCard = document.getElementById("phoneConnectCard");
const phoneQrImage = document.getElementById("phoneQrImage");
const phoneUrlLink = document.getElementById("phoneUrlLink");
const qrStatus = document.getElementById("qrStatus");

async function isLocalEmergencySession() {
  try {
    const response = await fetch(SESSION_ENDPOINT, { cache: "no-store" });
    if (!response.ok) return false;
    const payload = await response.json();
    return Boolean(payload?.local_emergency);
  } catch (error) {
    return false;
  }
}

async function initializePhoneQr() {
  if (!phoneConnectCard || !phoneQrImage || !phoneUrlLink || !qrStatus) return;

  phoneConnectCard.hidden = false;
  const localEmergency = await isLocalEmergencySession();
  if (!localEmergency) {
    window.location.replace("./index.html");
    return;
  }

  phoneQrImage.hidden = false;
  phoneUrlLink.hidden = false;
  try {
    const response = await fetch(CONNECT_INFO_ENDPOINT, { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);

    const info = await response.json();
    const phoneUrl = String(info.phone_url || window.location.origin + "/index.html");
    phoneUrlLink.href = phoneUrl;
    phoneUrlLink.textContent = phoneUrl;
    qrStatus.textContent = "Scan this QR code from your phone.";

    const localQrUrl = `${QR_ENDPOINT}?url=${encodeURIComponent(phoneUrl)}`;
    phoneQrImage.src = localQrUrl;
    phoneQrImage.alt = `QR code for ${phoneUrl}`;
    phoneQrImage.onerror = () => {
      const fallback = `https://api.qrserver.com/v1/create-qr-code/?size=220x220&data=${encodeURIComponent(phoneUrl)}`;
      phoneQrImage.src = fallback;
      qrStatus.textContent = "Using online QR fallback.";
    };
  } catch (error) {
    qrStatus.textContent = `QR unavailable: ${error.message}`;
  }
}

initializePhoneQr();
