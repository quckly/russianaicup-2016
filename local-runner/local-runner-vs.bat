cd /d %~dp0
start javaw -Xms512m -Xmx1G -XX:+UseConcMarkSweepGC -jar "local-runner.jar" local-runner.vs.properties local-runner.properties
