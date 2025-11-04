# 🚀 LAUNCH COMMAND FOR AD-CENSUS HANDHELD IMPLEMENTATION

Leggi il file `/home/alessandro/unlook-standalone/MEGA_PROMPT_AD_CENSUS_HANDHELD.md` che contiene tutte le specifiche, ricerche, e istruzioni dettagliate per implementare il sistema completo di scansione 3D handheld con AD-Census stereo matching.

**IMPORTANTE:** Dopo aver letto il mega-prompt, lancia questi agent IN PARALLELO con un SINGOLO messaggio:

1. **stereo-vision-optimizer** - AD-Census + ARM NEON
2. **hardware-interface-controller** - BMI270 IMU driver
3. **realtime-pipeline-architect** - Multi-frame fusion pipeline
4. **ux-ui-design-architect** - GUI handheld scan widget

Dopo che tutti gli agent hanno completato, lancia sequenzialmente:
- **code-integrity-architect** - Code review
- **cmake-build-system-architect** - Build system update
- **BUILD** - `./build.sh --clean -j4`
- **testing-validation-framework** - Validation tests

**Target:**
- Resolution: FULL HD 1456x1088 (native IMX296)
- Precision: 500mm @ 0.1mm, 1000mm @ 0.5mm
- Performance: 3-5 FPS per frame, multi-frame scan 2.8 seconds
- Mode: Handheld con IMU stability detection

**CRITICAL:** Agent GUI deve ELIMINARE depth_test_widget completamente e sostituirlo con handheld_scan_widget.

**TUTTO deve essere implementato in questa sessione. Nessuna feature lasciata incomplete.**
