# Calibration methods: what exists and what we build

Reviewed 2026-11-15. The question: how to find, for each IMU, its mounting on the body segment
and its private heading origin, with less user effort and less sensitivity to pose execution than
the current neutral + held skiing pose ([functional-mounting.md](functional-mounting.md)), both
with a live headset and without one.

**Legend.** **[V]** read on the primary page or full text; **[A]** abstract or record only;
**[U]** unverified (snippets or third-party pages). Accuracy figures are the authors' on their own
data. Nothing here is a measurement on this hardware.

## What products ask users to do

| System | Procedure | Known pain points |
| --- | --- | --- |
| SlimeVR [V] ([setup](https://docs.slimevr.dev/quick-setup.html)) | Full reset in I-pose, yaw reset, mounting reset in a held "ski" pose. v22 release candidates add **step mounting**: one step forward, each tracker's acceleration double-integrated with a final-rest bias fit, yaw-aligned to the headset's displacement | Calibration was the top complaint in a project poll (45 %) ([update 239](https://slimevr.dev/blog/weekly-update-239/)); issue #1869 "Mounting calibration extremely difficult to impossible"; step mounting fails on firmware without the corrected acceleration frame (issue #1882) |
| Sony mocopi [V] ([guide](https://helpguide.sony.net/mobile/qm-ss1/v1/en/contents/calibration.html)) | Stand straight, take one vigorous step forward, stand straight again | Fails with too little momentum or movement while standing |
| Xsens / Movella MVN [V] (user manual, rev. Z) | Hold N-pose or T-pose 5–10 s, then walk back and forth 5–10 m; plain static poses are hidden as "not recommended"; quality is graded | Needs walking space |
| GlobalPose (SIGGRAPH 2025) [V] ([arXiv](https://arxiv.org/abs/2505.05010)) | Stand, one step forward; each IMU's integrated horizontal displacement gives its relative heading, the displacement defines forward, the standing pose gives mounting | Evaluated by user preference only (72 % over T-pose); no angular error reported |
| Rokoko, Perception Neuron, HaritoraX [U] | Static poses | Not verified |

The field has moved from held poses to **a pose plus a natural movement**, and step-based heading
alignment is the current direction. It depends on trustworthy world-frame acceleration.

## Functional calibration in the literature

- **Seel, Raisch & Schauer, Sensors 2014** [V] ([doi](https://doi.org/10.3390/s140406891)):
  knee axis in both sensor frames from angular rates of arbitrary motion with a dominant hinge.
  The sign is resolved "by a quick look at the sensor's mounting", that is, manually. Reported
  knee flexion RMSE about 3° on a human leg.
- **McGrath, Fineman & Stirling, Sensors 2018** [V] ([doi](https://doi.org/10.3390/s18061882)):
  knee axis as the principal component of relative angular velocity, no discrete calibration
  motion. PCA fits a line, so it gives no sign. Reported RMSE 9.24° absolute, 3.49° zero-mean,
  and sensitivity to thigh sensor placement.
- **Fasel et al., PLoS ONE 2017** [V] ([doi](https://doi.org/10.1371/journal.pone.0181446)):
  slow squats give the medio-lateral axis of shank, thigh, lower back and sternum; trunk rotation
  gives the vertical axis; upright standing gives the rest. Reported repeatability under 2.7°,
  accuracy −10.7° to +4.2° with a subject-specific flexion bias.
- **Nazarahari et al., J. Biomech 2019** [A]: quiet standing plus ten hip flexions gave the most
  repeatable calibration among those compared. A companion paper uses standing plus eight steps.
- **Robert-Lachaine et al., Gait & Posture 2017** [A]: static poses repeat to about 1.5–2.1°.
- **Reviews** (Pacher et al. 2020; Vitali & Perkins 2020) [A]: no method is established as best;
  estimates differ substantially between and within classes of method.
- **TIC, Zuo et al., SIGGRAPH 2025** [V] ([arXiv](https://arxiv.org/abs/2506.10580)): a
  transformer that continuously estimates heading drift and bone-to-sensor offset, gated by a
  "rotation diversity" measure. It does not correct global yaw. Transferable without training:
  the diversity gate and the separation of left-side drift from right-side offset, which this
  project already keeps.
- **Slip detection:** no validated non-learned online method was found.

## Human factors that bias calibration

- Preferred stance is not feet-parallel: about 14° between the feet (McIlroy & Maki, Clin.
  Biomech. 1997 [A]), so each leg's sagittal plane sits several degrees off the body's.
- Hip and trunk flexion ranges far exceed extension, and the knee flexes one way only. These
  asymmetries can decide forward versus backward without an instruction.
- No direct comparison of squat, bow, leg swing and walking repeatability was found.

## What this means for ChadFBT

**The acceleration frame is the gate for step-based methods.** Firmware 0.7.2 sends acceleration
that is not rotated like the quaternion, which is exactly the condition under which SlimeVR's own
step mounting is reported to fail. Step calibration is therefore not the first thing to build
here; the [acceleration-heading](acceleration-heading.md) module's yaw-independent frame
identification is the prerequisite, and it is still a shadow estimator.

**Built: motion calibration** ([motion-calibration.md](motion-calibration.md)). After the neutral
capture, any mostly-sagittal movement is accumulated continuously (a bow, squats, marching in
place, or the old skiing pose, held or not). Each sensor's fore-aft line is fitted from the whole
trajectory of its bone-axis tip, the sign comes from one-sided anatomy, and every segment reports
its own progress and blocker. Cross-checks name a mid-procedure turn or apparently swapped
trackers. This works with or without a headset.

**Built: displacement-referenced forward.** With a headset, the horizontal displacement of the
neck pivot during the forward bend gives body-forward in the room frame, fused with the headset's
yaw at neutral. The user no longer has to hold the head exactly in line with the torso.

**Refinement after calibration is already continuous.** With a headset the stance heading filter
corrects leg heading from displacement during ordinary movement, so a calibration that is a few
degrees off (knees pointing outward in the squat, natural toe-out) is pulled in without another
procedure. After motion calibration the filter starts with wider uncertainty for exactly that
reason.

**Prior art, plainly.** "Neutral, then a functional sagittal movement to find the medio-lateral
axis" is published (Fasel 2017, Nazarahari 2019), and so is PCA for a hinge axis (McGrath 2018).
Resolving the sign automatically from range-of-motion asymmetry was not found in the sources
reviewed, and neither was using headset displacement during a *bend* (as opposed to a step) as the
forward reference; patents were not searched. We describe both as combinations and variants of
published calibration elements, not as new methods.

## Pitfalls and the checks that catch them

| Pitfall | Check |
| --- | --- |
| Head not aligned with torso | Use displacement, not yaw; flag disagreement above 25° |
| Knees out, natural toe-out | Left/right deviations that are symmetric and opposite indicate posture; report, do not force parallel; let the stance filter refine |
| 180° flips | One-sided anatomy and the knee rule; never a silent default |
| Too little or non-sagittal motion | Minimum tilt, planarity, repeatability between interleaved halves; name the failing segment |
| Turning during the procedure | The fore-aft lines before and after disagree |
| Swapped trackers | Knee hyperextension or inconsistent hinge axes after the solve |
| Circular validation | Check the result with an independent motion; record live validation separately |
| Strap slip later in the session | Rising knee-hinge residual; prompt, do not silently re-mount |

## Next experiments for calibration

1. **Step calibration with frame self-identification.** Double-integrate world-frame acceleration
   over a step between two rests, with the acceleration frame chosen by the yaw-independent
   vertical check (a wrong frame turns a level step into vertical displacement). It gives every
   sensor's heading, torso included, with no assumption about how the legs swing, and is an
   independent cross-check of motion calibration. Noisy by nature: a 0.15 m/s² residual bias is
   about 17 cm over a 1.5 s step, so several steps must be averaged.
2. **Calibration without an explicit neutral.** Detect upright stillness from headset height and
   sensor stability. The risk is asymmetric standing, so an auto-detected neutral should only be
   allowed to make bounded heading updates, never to re-derive mounting.
3. **Twist-only quick fix** for a strap that rotated: re-solve one segment's mounting twist from a
   single knee bend, leaving everything else untouched.
