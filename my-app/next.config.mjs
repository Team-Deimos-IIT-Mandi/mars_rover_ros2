/** @type {import('next').NextConfig} */
const nextConfig = {
    output: 'standalone', // Creates self-contained build for deployment
    compress: true,
    reactStrictMode: true,
    swcMinify: true,
    experimental: {
        optimizePackageImports: ['lucide-react', '@radix-ui']
    }
};

export default nextConfig;
