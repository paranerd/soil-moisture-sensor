let form = document.getElementById("form");
let msg = document.getElementById("msg");

form.addEventListener("submit", () => {
  console.log("submitting...");

  msg.classList.remove("hidden");
});
