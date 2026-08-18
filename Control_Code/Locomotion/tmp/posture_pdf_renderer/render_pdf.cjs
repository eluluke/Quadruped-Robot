const path = require('path');
const fs = require('fs');
const http = require('http');
const { chromium } = require('playwright');

async function main() {
  const input = path.resolve(__dirname, 'posture_theory_revised.html');
  const output = path.resolve(
    __dirname,
    '..',
    '..',
    'output',
    'pdf',
    'Quadruped_Posture_Correction_Theory_Revised.pdf'
  );

  const server = http.createServer((request, response) => {
    const requestPath = decodeURIComponent(request.url.split('?')[0]);
    const relativePath = requestPath === '/' ? 'posture_theory_revised.html' : requestPath.slice(1);
    const filePath = path.resolve(__dirname, relativePath);

    if (!filePath.startsWith(__dirname)) {
      response.writeHead(403);
      response.end('Forbidden');
      return;
    }

    fs.readFile(filePath, (error, data) => {
      if (error) {
        console.error(`server 404: ${filePath} (${error.code})`);
        response.writeHead(404);
        response.end('Not found');
        return;
      }

      const extension = path.extname(filePath).toLowerCase();
      const contentTypes = {
        '.css': 'text/css',
        '.html': 'text/html; charset=utf-8',
        '.js': 'text/javascript',
        '.woff': 'font/woff',
        '.woff2': 'font/woff2'
      };
      response.writeHead(200, { 'Content-Type': contentTypes[extension] || 'application/octet-stream' });
      response.end(data);
    });
  });

  await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
  const address = server.address();

  const browser = await chromium.launch({
    channel: 'chrome',
    headless: true,
    args: ['--allow-file-access-from-files', '--disable-gpu']
  });

  try {
    const page = await browser.newPage({ viewport: { width: 1224, height: 1584 } });
    page.on('console', (message) => console.log(`browser: ${message.type()}: ${message.text()}`));
    page.on('pageerror', (error) => console.error(`browser page error: ${error.message}`));
    await page.goto(`http://127.0.0.1:${address.port}/${path.basename(input)}`, {
      waitUntil: 'networkidle'
    });
    await page.waitForFunction(() => document.querySelectorAll('.katex').length > 20, null, {
      timeout: 5000
    });
    await page.evaluate(() => document.fonts.ready);

    const qa = await page.evaluate(() => ({
      formulaErrors: document.querySelectorAll('.katex-error').length,
      overflowingPages: Array.from(document.querySelectorAll('.page'))
        .map((element, index) => ({
          page: index + 1,
          overflowX: element.scrollWidth - element.clientWidth,
          overflowY: element.scrollHeight - element.clientHeight
        }))
        .filter((item) => item.overflowX > 1 || item.overflowY > 1)
    }));

    if (qa.formulaErrors || qa.overflowingPages.length) {
      throw new Error(`Layout QA failed: ${JSON.stringify(qa)}`);
    }

    await page.emulateMedia({ media: 'print' });
    await page.pdf({
      path: output,
      format: 'Letter',
      printBackground: true,
      preferCSSPageSize: true,
      margin: { top: '0', right: '0', bottom: '0', left: '0' }
    });
  } finally {
    await browser.close();
    await new Promise((resolve) => server.close(resolve));
  }
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
