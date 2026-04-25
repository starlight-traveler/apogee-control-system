(function () {
    function qsa(selector, root) {
        return Array.prototype.slice.call((root || document).querySelectorAll(selector));
    }

    function setText(id, value) {
        var el = document.getElementById(id);
        if (el) {
            el.textContent = value;
        }
    }

    qsa('.reveal').forEach(function (el, index) {
        el.style.transitionDelay = Math.min(index * 40, 240) + 'ms';
    });

    if ('IntersectionObserver' in window) {
        var observer = new IntersectionObserver(function (entries) {
            entries.forEach(function (entry) {
                if (entry.isIntersecting) {
                    entry.target.classList.add('visible');
                    observer.unobserve(entry.target);
                }
            });
        }, { threshold: 0.08 });
        qsa('.reveal').forEach(function (el) { observer.observe(el); });
    } else {
        qsa('.reveal').forEach(function (el) { el.classList.add('visible'); });
    }

    qsa('[data-phase-target]').forEach(function (button) {
        button.addEventListener('click', function () {
            var target = button.getAttribute('data-phase-target');
            qsa('[data-phase-target]').forEach(function (btn) {
                btn.classList.toggle('active', btn === button);
                btn.setAttribute('aria-selected', btn === button ? 'true' : 'false');
            });
            qsa('[data-phase]').forEach(function (phase) {
                phase.dataset.active = phase.getAttribute('data-phase') === target ? 'true' : 'false';
            });
            qsa('[data-phase-copy]').forEach(function (copy) {
                copy.hidden = copy.getAttribute('data-phase-copy') !== target;
            });
        });
    });

    function updatePredictorDemo() {
        var altitude = Number((document.getElementById('demo-altitude') || {}).value || 420);
        var velocity = Number((document.getElementById('demo-velocity') || {}).value || 210);
        var flap = Number((document.getElementById('demo-flap') || {}).value || 0);
        var tilt = Number((document.getElementById('demo-tilt') || {}).value || 4);
        var g = 9.80665;
        var dragFactor = 1.0 + flap * 0.018 + tilt * 0.008;
        var ballistic = altitude + (velocity * velocity) / (2 * g);
        var predicted = altitude + (velocity * velocity) / (2 * g * dragFactor);
        var lost = ballistic - predicted;
        var tta = velocity / (g * Math.sqrt(dragFactor));

        setText('demo-altitude-value', altitude.toFixed(0) + ' m');
        setText('demo-velocity-value', velocity.toFixed(0) + ' m/s');
        setText('demo-flap-value', flap.toFixed(0) + ' deg');
        setText('demo-tilt-value', tilt.toFixed(0) + ' deg');
        setText('demo-apogee', predicted.toFixed(0) + ' m');
        setText('demo-time', tta.toFixed(1) + ' s');
        setText('demo-drag-loss', lost.toFixed(0) + ' m');

        var fill = document.getElementById('demo-bar-fill');
        if (fill) {
            fill.style.width = Math.max(0, Math.min(100, (flap / 45) * 100)) + '%';
        }
    }

    qsa('[data-predictor-input]').forEach(function (input) {
        input.addEventListener('input', updatePredictorDemo);
    });
    updatePredictorDemo();

    function updateMathDemo() {
        var vz = Number((document.getElementById('math-vz') || {}).value || 180);
        var accel = Number((document.getElementById('math-drag') || {}).value || 12);
        var g = 9.80665;
        var ballistic = (vz * vz) / (2 * g);
        var draggy = (vz * vz) / (2 * Math.max(g + accel, 0.1));
        var loss = ballistic - draggy;
        var effectiveG = 1 + accel / g;

        setText('math-vz-value', vz.toFixed(0) + ' m/s');
        setText('math-drag-value', accel.toFixed(1) + ' m/s^2');
        setText('math-ballistic', ballistic.toFixed(0) + ' m');
        setText('math-with-drag', draggy.toFixed(0) + ' m');
        setText('math-loss', loss.toFixed(0) + ' m');
        setText('math-effective-g', effectiveG.toFixed(2) + ' g');
    }

    qsa('[data-math-input]').forEach(function (input) {
        input.addEventListener('input', updateMathDemo);
    });
    updateMathDemo();

    qsa('[data-rail-toggle]').forEach(function (button) {
        button.addEventListener('click', function () {
            var rail = button.getAttribute('data-rail-toggle');
            qsa('[data-rail-toggle]').forEach(function (btn) {
                btn.classList.toggle('active', btn === button);
                btn.setAttribute('aria-selected', btn === button ? 'true' : 'false');
            });
            qsa('[data-rail-detail]').forEach(function (panel) {
                panel.hidden = panel.getAttribute('data-rail-detail') !== rail;
            });
        });
    });
})();
