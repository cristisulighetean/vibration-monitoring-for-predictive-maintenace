/* Generated by Edge Impulse
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#ifndef _EI_CLASSIFIER_ANOMALY_CLUSTERS_H_
#define _EI_CLASSIFIER_ANOMALY_CLUSTERS_H_

#include "edge-impulse-sdk/anomaly/anomaly.h"

// (before - mean) / scale
const float ei_classifier_anom_scale[EI_CLASSIFIER_ANOM_AXIS_SIZE] = { 0.05585386820475774, 0.03728605054276337, 0.07047022717759732 };
const float ei_classifier_anom_mean[EI_CLASSIFIER_ANOM_AXIS_SIZE] = { 0.055265586277515114, 0.050971075139687794, 0.08195726661638512 };

const ei_classifier_anom_cluster_t ei_classifier_anom_clusters[EI_CLASSIFIER_ANOM_CLUSTER_COUNT] = { { { -0.5234009027481079, -0.1968267410993576, -0.5200677514076233 }, 0.6209319799889041 }
, { { -0.09248296916484833, 0.2449236512184143, -0.006933869794011116 }, 0.39127499307653946 }
, { { -0.01970953308045864, -0.536055862903595, -0.6321200132369995 }, 0.4395852304256389 }
, { { -0.13176754117012024, 0.23684155941009521, 0.4710296392440796 }, 0.4300331152980091 }
, { { -0.9526997208595276, -1.3230392932891846, -1.1386419534683228 }, 0.15810462920910792 }
, { { 0.8803512454032898, -0.25085780024528503, 0.9687160849571228 }, 0.5866664062809738 }
, { { -0.38171568512916565, 0.231694296002388, -0.33328887820243835 }, 0.5444104285147685 }
, { { 0.09147254377603531, -0.048557061702013016, -0.2380286157131195 }, 0.4306560520919586 }
, { { -0.23347337543964386, 0.802602231502533, 0.6260737776756287 }, 0.49277180355499184 }
, { { 2.5648958683013916, 0.976250410079956, 2.0145857334136963 }, 0.5028416880241838 }
, { { -0.6266619563102722, -0.6507214307785034, -0.42353957891464233 }, 0.45996194313477345 }
, { { 1.0846844911575317, -0.7012233734130859, 1.1301473379135132 }, 0.4288103122231278 }
, { { 0.19928273558616638, 1.6186339855194092, -0.13930658996105194 }, 0.6775237840682565 }
, { { -0.39671996235847473, 0.3266209065914154, 0.1404789686203003 }, 0.37997531323558575 }
, { { 2.5073647499084473, 1.4299932718276978, 2.241245985031128 }, 0.4911207217804007 }
, { { -0.13204507529735565, 1.320106029510498, 1.0134906768798828 }, 1.0583330498625119 }
, { { 2.376800060272217, 2.384838581085205, 2.0217833518981934 }, 0.8647352048893846 }
, { { -0.3650018274784088, -0.3327999413013458, 0.3622335195541382 }, 0.5019150922033871 }
, { { -0.16101926565170288, -0.12908577919006348, 0.8188135623931885 }, 0.8229967008014791 }
, { { 0.5399972796440125, -0.008387109264731407, 0.5411702394485474 }, 0.5403336571635674 }
, { { 0.09023962169885635, 0.6406068801879883, -0.2528267204761505 }, 0.4281591643053965 }
, { { 2.719146966934204, 1.8636257648468018, 2.4410152435302734 }, 0.5434770215426524 }
, { { 0.12420942634344101, -0.19192101061344147, -0.5779711008071899 }, 0.48236983625278756 }
, { { 0.641170084476471, 0.4056665599346161, 0.7462396621704102 }, 0.8443190109306239 }
, { { -0.012627501040697098, 0.9924857020378113, -0.3972298502922058 }, 0.41074592785375985 }
, { { -0.21332991123199463, 0.7902774810791016, 0.055445220321416855 }, 0.5016890814254078 }
, { { -0.39675477147102356, -0.21658864617347717, -0.1457817405462265 }, 0.3488180772691182 }
, { { -0.4500574469566345, -0.6693742871284485, 0.062423210591077805 }, 0.4875449628383771 }
, { { 2.2697904109954834, 1.7364568710327148, 2.056366205215454 }, 0.49679176852825907 }
, { { 2.5999207496643066, 0.9728775024414062, 2.4429209232330322 }, 0.5021451105555208 }
, { { 1.0133756399154663, 0.38362497091293335, 1.159696340560913 }, 0.464470346796196 }
, { { 0.16856782138347626, 3.0655858516693115, 2.196678638458252 }, 0.8553896630132976 }
};

#endif // _EI_CLASSIFIER_ANOMALY_CLUSTERS_H_
