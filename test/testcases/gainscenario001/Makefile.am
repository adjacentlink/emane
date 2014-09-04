harness_dir=$(top_srcdir)/test/harness/gainscenario
harness_cmd=gainscenario
harness_exe=$(harness_dir)/$(harness_cmd)
harness_bin_deps=$(harness_exe) $(top_srcdir)/src/libemane/.libs/libemane.so
harness_schema=gainscenario.xsd

testcase_inputs=      \
 antenna30dsector.xml \
 antennaomni.xml      \
 antennahole.xml      \
 antennaprofiles.xml  \
 blockageaft.xml      \
 blockagehole.xml     \
 gainscenario.xml

EXTRA_DIST=           \
 $(testcase_inputs)   \
 testcase-target.txt

all-local: testcase-output.txt
	@diff -q testcase-output.txt testcase-target.txt &> /dev/null

testcase-output.txt: $(testcase_inputs) $(harness_bin_deps)
	$(harness_exe) \
    --schema $(harness_dir)/$(harness_schema) \
    antennaprofiles.xml gainscenario.xml &> testcase-output.txt

clean-local:
	rm -f testcase-output.txt
